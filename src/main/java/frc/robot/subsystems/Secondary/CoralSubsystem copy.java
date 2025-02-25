// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// /*
//  * This is the CoralSubsystem class. This class is used to control the arm that processes the coral.
//  * The arm has four systems to control.
//  * 
//  * The first is the rotation of the arm which is driven
//  * by a neoVortex motor with a SparkFlex controller. We will refer to this as coralRotate.
//  * 
//  * The second is the intake / launch system which is driven by a neo550 motor with a SparkMax control.
//  * There is also a beam brake sensor to detect the presence of a coral.
//  * We will refer to this system as the coralLaunch.
//  * 
//  * The thrid is system that moves the coral side to side to allign it with the reef. On the right 
//  * side of the robot is a limit switch that is used to initialize the system to a known position.
//  * This system is driven by a neo550 motor with a SparkMax controller. We will refer to this as coralAlign.
//  * The side to side position will need to be determined based on which side of the reef we are
//  * trying to score on, compared to the location of the robot relative to the AprilTag.
//  * 
//  * The fourth system is a servo that is used to place the coral intake funnel into the match position
//  * which is required to have the funnel start the match inside the bumper zone.
//  */

// package frc.robot.subsystems.Secondary;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// //import com.revrobotics.CANDigitalInput;
// import com.revrobotics.sim.SparkAbsoluteEncoderSim;
// import com.revrobotics.sim.SparkMaxSim;
// import com.revrobotics.sim.SparkRelativeEncoderSim;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// // import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import frc.robot.Robot;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants.CoralConstants;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Servo;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;                                                           

// public class CoralSubsystem extends SubsystemBase {

//     private SparkMax coralAngMtr;
//     public AbsoluteEncoder coralAngEnc;
//     public SparkClosedLoopController  coralAngPID;
//     private SparkMax coralSldrMtr;
//     public RelativeEncoder coralSldrEnc;
//     public SparkMax coralIndexMtr;
//     public RelativeEncoder coralIndexEnc;
//     public SparkClosedLoopController coralSldrPID;
//     public Servo pusherServo;
//     private SparkMaxSim coralAngMtrSim;
//     private SparkMaxSim coralSldrMtrSim;
//     private SparkMaxSim coralIndexMtrSim;
//     private SparkAbsoluteEncoderSim coralAngEncSim; 
//     private SparkRelativeEncoderSim coralSldrEncSim;
//     private SparkRelativeEncoderSim coralIndexEncSim;
//     private SparkMaxConfig coralAngMtrCfg;
//     private SparkMaxConfig coralSldrMtrCfg;
//     private SparkMaxConfig coralIndexMtrCfg;
//     private DigitalInput coralSensor;
//     // private SoftLimitConfig rotateMtrSftLmtCfg;.
//     public SparkLimitSwitch coralLimitSwitch;
    
//     private double angkP = 0.005, angkI = 0.0, angkD = 0.0;//p was 0.0005
//     private double angkFF = 0.0;
//     private double angOutputMin = 0.0;
//     private double angOutputMax = 0.3;

//     private double sldrkP = 0.005, sldrkI = 0.0, sldrkD = 0.0;//p was 0.0005
//     private double sldrkFF = 0.0;
//     private double sldrkOutputMin = 0.0;
//     private double sldrkOutputMax = 0.3;

//     private boolean sliderInitialized;

//     public CoralSubsystem() {
//         coralAngMtr = new SparkMax(CoralConstants.CORAL_ROTATE_MOTOR_PORT, MotorType.kBrushless);
//         coralSldrMtr = new SparkMax(CoralConstants.CORAL_SLIDER_MOTOR_PORT, MotorType.kBrushless);
//         coralIndexMtr = new SparkMax(CoralConstants.CORAL_HOLD_MOTOR_PORT, MotorType.kBrushless);
//         pusherServo = new Servo(CoralConstants.SERVO_PORT);
//         coralAngMtrCfg = new SparkMaxConfig();
//         coralSldrMtrCfg = new SparkMaxConfig();
//         coralIndexMtrCfg = new SparkMaxConfig();
//         coralSensor = new DigitalInput(CoralConstants.BEAM_BREAK_SENSOR_PORT);
//         coralLimitSwitch = coralSldrMtr.getForwardLimitSwitch();
//         // encCfg = new AbsoluteEncoderConfig();
//         // rotateMtrSftLmtCfg = new SoftLimitConfig();

//         coralAngPID = coralAngMtr.getClosedLoopController();
//         coralAngEnc = coralAngMtr.getAbsoluteEncoder();
//         coralSldrPID = coralSldrMtr.getClosedLoopController();
//         coralSldrEnc = coralSldrMtr.getEncoder();
//         coralIndexEnc = coralIndexMtr.getEncoder();
 
//         coralAngMtrCfg
//             .inverted(true)
//             .voltageCompensation(12.0)
//             .smartCurrentLimit(40)
//             .idleMode(IdleMode.kBrake);
//         coralAngMtrCfg
//             .absoluteEncoder
//                 .positionConversionFactor(360);
//         coralAngMtrCfg
//             .softLimit
//                 .forwardSoftLimit(150.0) 
//                 .reverseSoftLimit(290.0);
//         coralAngMtrCfg
//             .closedLoop
//                 .pidf(angkP, angkI, angkD, angkFF)
//                 .outputRange(angOutputMin, angOutputMax)
//                 .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
//                 // .maxMotion
//                 //     .allowedClosedLoopError(2.0);   
//                 //     .maxAcceleration(kMaxAccel)
//                 //     .maxVelocity(kMaxRPM)
//                 //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
//         coralAngMtr.configure(coralAngMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


//         coralSldrMtrCfg
//             .inverted(true)
//             .voltageCompensation(12.0)
//             .smartCurrentLimit(40)
//             .idleMode(IdleMode.kBrake);
//         coralSldrMtrCfg
//             .encoder
//                 .positionConversionFactor(360);//TO DO change to inches
//         coralSldrMtrCfg
//             .softLimit
//                 .forwardSoftLimit(150.0) 
//                 .reverseSoftLimit(290.0);
//         coralSldrMtrCfg
//             .limitSwitch
//                 .forwardLimitSwitchEnabled(true);
//         coralSldrMtrCfg
//             .closedLoop
//                 .pidf(sldrkP, sldrkI, sldrkD, sldrkFF)
//                 .outputRange(sldrkOutputMin, sldrkOutputMax)
//                 .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
//                 // .maxMotion
//                 //     .allowedClosedLoopError(2.0);   
//                 //     .maxAcceleration(kMaxAccel)
//                 //     .maxVelocity(kMaxRPM)
//                 //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
//         coralSldrMtr.configure(coralSldrMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
//         coralIndexMtrCfg
//             .inverted(true)
//             .voltageCompensation(12.0)
//             .smartCurrentLimit(40)
//             .idleMode(IdleMode.kBrake);
//         coralSldrMtr.configure(coralIndexMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           
//         // Add motors to the simulation
//         if (Robot.isSimulation()) {
//             coralAngMtrSim = new SparkMaxSim(coralAngMtr, DCMotor.getNEO(1));
//             coralAngEncSim = new SparkAbsoluteEncoderSim(coralAngMtr);
//             coralAngMtrSim.setPosition(190);
//             coralAngEncSim.setPosition(190);
//             coralAngMtrSim.setVelocity(0);
//             coralAngEncSim.setVelocity(0);
//             coralSldrMtrSim = new SparkMaxSim(coralSldrMtr, DCMotor.getNEO(1));
//             coralSldrEncSim = new SparkRelativeEncoderSim(coralSldrMtr);
//             coralIndexMtrSim = new SparkMaxSim(coralIndexMtr, DCMotor.getNEO(1));
//             coralIndexEncSim = new SparkRelativeEncoderSim(coralIndexMtr);
//         }
//     }
    
//     // public void runIntakeCmd(double speed) {
//     //     coralIndexMtr.set(speed);
//     // }

//     /**
//      * Creates a FunctionalCommand to initialize the slider mechanism.
//      * 
//      * The command performs the following actions:
//      * 1. Initializes the sliderInitialized flag to false.
//      * 2. Continuously checks the state of the limit switch:
//      *    - If the limit switch is not triggered, the coralSliderMtr motor is set to move the slider.
//      *    - If the limit switch is triggered, the coralSliderMtr motor is stopped and the sliderInitialized flag is set to true.
//      * 3. Stops the coralSliderMtr motor if the command is interrupted.
//      * 4. Ends the command when the sliderInitialized flag is true.
//      * 
//      * @return A new FunctionalCommand instance for initializing the slider.
//      */
//     public FunctionalCommand SliderInitCmd() {
//         return new FunctionalCommand(() -> sliderInitialized = false,
//                                      () -> {if(!coralLimitSwitch.isPressed()) {
//                                                 coralSldrMtr.set(-.125);
//                                                 //I think this can just be an else statement
//                                             } else if(coralLimitSwitch.isPressed()) {
//                                                 coralSldrMtr.set(0);
//                                                 coralIndexEnc.setPosition(0);
//                                                 sliderInitialized = true;
//                                             }},
//                                      interrupted ->   coralSldrMtr.set(0),
//                                      () -> sliderInitialized,
//                                      this);
//     }

//     /**
//      * Creates a new FunctionalCommand for the intake mechanism.
//      * 
//      * The command performs the following actions:
//      * - Initializes with an empty lambda function.
//      * - Executes the intake mechanism with a speed of 0.1.
//      * - Ends the intake mechanism by stopping it (setting speed to 0) when interrupted.
//      * - Checks if the beam break sensor is triggered to determine if the command is finished.
//      * 
//      * @return A new instance of FunctionalCommand for controlling the intake mechanism.
//      */
//     public FunctionalCommand IntakeCmd(double speed) {
//         return new FunctionalCommand(() ->{},
//                                      () -> coralIndexMtr.set(speed),
//                                      interrupted -> coralIndexMtr.set(0),
//                                      () -> coralSensor.get(),
//                                      this);
//     }

//     public FunctionalCommand OuttakeCmd(double speed) {
//         return new FunctionalCommand(() ->{},
//                                      () -> coralIndexMtr.set(speed),
//                                      interrupted -> coralIndexMtr.set(0),
//                                      () -> !coralSensor.get(),
//                                      this);
//     }

//     public FunctionalCommand setRotateAngleCmd(double pos) {
//         return new FunctionalCommand(
//             () -> {},
//             () -> coralSldrPID.setReference(pos, SparkMax.ControlType.kPosition),
//             interrupted -> {},
//             () -> (Math.abs(pos - coralAngEnc.getPosition()) <= 5.0) && (Math.abs(coralAngEnc.getVelocity()) <= 60.0),
//             this);
//     }

//     public FunctionalCommand setSliderPosition(double pos) {
//         return new FunctionalCommand(() -> {},
//             () -> coralSldrPID.setReference(pos, SparkMax.ControlType.kPosition),
//             interrupted -> {},
//             () -> (Math.abs(pos - coralSldrEnc.getPosition()) <= 5.0) && (Math.abs(coralSldrEnc.getVelocity()) <= 60.0),
//             this);
//     }

    
//     // public void setRotateAngle(double angle) {
//     //     coralAngPID.setReference(angle, SparkMax.ControlType.kPosition);
//     //     // if (Robot.isSimulation()) {
//     //     //     coralRotatePID.setReference(angle, SparkMax.ControlType.kPosition);
//     //     // }
//     // }

//     // public void setSliderPosition(double position) {
//     //     coralSldrPID.setReference(position, SparkMax.ControlType.kPosition);
//     //     // if (Robot.isSimulation()) {
//     //     //     coralSliderPID.setReference(position, SparkMax.ControlType.kPosition);
//     //     // }
//     // }


//     // public void extend() {
//     //     pusherServo.set(1);
//     // }

//     @Override
//     public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//         coralAngEncSim.setPosition(coralAngMtrSim.getPosition());
//         coralAngMtrSim.iterate(coralAngEncSim.getPosition(), coralAngMtrSim.getBusVoltage(),.005);

//         coralSldrEncSim.setPosition(coralSldrMtrSim.getPosition());
//         coralSldrMtrSim.iterate(coralSldrEncSim.getPosition(), coralSldrMtrSim.getBusVoltage(),.005);
        
//         coralIndexEncSim.setPosition(coralIndexMtrSim.getPosition());
//         coralIndexMtrSim.iterate(coralIndexEncSim.getPosition(), coralIndexMtrSim.getBusVoltage(), .005);
//     }
    
//     @Override
//     public void periodic() {
//     // This method will be called once per scheduler run
//     if (Robot.isSimulation()) {
//         SmartDashboard.putNumber("Coarl Arm Position", coralAngEncSim.getPosition());
//         SmartDashboard.putNumber("Coral Slider Position", coralSldrEncSim.getPosition());
//         SmartDashboard.putNumber("Coral Holder Speed", coralIndexMtrSim.getVelocity());
//     } else {
//         SmartDashboard.putNumber("Coral Arm Position", coralAngEnc.getPosition());
//         SmartDashboard.putNumber("Coral Slider Position", coralSldrEnc.getPosition());
//         SmartDashboard.putNumber("Coral Holder Speed", coralIndexEnc.getVelocity());
//     }

//     }
// }
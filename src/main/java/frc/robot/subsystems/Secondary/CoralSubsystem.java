// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * This is the CoralSubsystem class. This class is used to control the arm that processes the coral.
 * The arm has four systems to control.
 * 
 * The first is the rotation of the arm which is driven
 * by a neoVortex motor with a SparkFlex controller. We will refer to this as coralRotate.
 * 
 * The second is the intake / launch system which is driven by a neo550 motor with a SparkMax control.
 * There is also a beam brake sensor to detect the presence of a coral.
 * We will refer to this system as the coralLaunch.
 * 
 * The thrid is system that moves the coral side to side to allign it with the reef. On the right 
 * side of the robot is a limit switch that is used to initialize the system to a known position.
 * This system is driven by a neo550 motor with a SparkMax controller. We will refer to this as coralAlign.
 * The side to side position will need to be determined based on which side of the reef we are
 * trying to score on, compared to the location of the robot relative to the AprilTag.
 * 
 * The fourth system is a servo that is used to place the coral intake funnel into the match position
 * which is required to have the funnel start the match inside the bumper zone.
 */

package frc.robot.subsystems.Secondary;

// import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANDigitalInput;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;                                                           

public class CoralSubsystem extends SubsystemBase {

    private SparkFlex coralAngMtr;
    public AbsoluteEncoder coralAngEnc;
    public SparkClosedLoopController  coralAngPID;
    // public SparkClosedLoopController coralIndexPID;
    // private SparkMax coralSldrMtr;
    // public RelativeEncoder coralSldrEnc;
    public SparkFlex outtakeMtrLdr;
    public SparkFlex outtakeMtrFlw;
    public RelativeEncoder outtakeMtrLdrEnc;
    public RelativeEncoder outtakeMtrFlwEnc;
    // public SparkClosedLoopController coralSldrPID;
    // public Servo pusherServo;
    private SparkFlexSim coralAngMtrSim;
    //private SparkMaxSim coralSldrMtrSim;
    private SparkFlexSim outtakeMtrLdrSim;
    private SparkFlexSim outtakeMtrFlwSim;
    private SparkAbsoluteEncoderSim coralAngEncSim; 
    //private SparkRelativeEncoderSim coralSldrEncSim;
    private SparkRelativeEncoderSim outtakeMtrLdrEncSim;
    private SparkRelativeEncoderSim outtakeMtrFlwEncSim;
    private SparkFlexConfig coralAngMtrCfg;
    //private SparkMaxConfig coralSldrMtrCfg;
    private SparkFlexConfig outtakeMtrLdrCfg;
    private SparkFlexConfig outtakeMtrFlwCfg;
    private DigitalInput coralSensor;
    // private SoftLimitConfig rotateMtrSftLmtCfg;.
    public SparkLimitSwitch coralLimitSwitch;

    private boolean coralAngleInitialized;
    
    private double angkP = 0.0175, angkI = 0.0, angkD = 0.0;//p was 0.0005
    private double outtakekP = 0.125, outtakekI = 0.0, outtakekD = 0.0;//p was 0.0005
    private double angkFF = 0.0;
    private double angOutputMin = -1.0;
    private double angOutputMax = 1.0;
    private double outtakeOutputMin = -1.0;
    private double outtakeOutputMax = 1.0;
    public boolean close;

    // public final CANrange canrange = new CANrange(29);

    
    // private double sldrkP = 0.005, sldrkI = 0.0, sldrkD = 0.0;//p was 0.0005
    // private double sldrkFF = 0.0;
    // private double sldrkOutputMin = 0.0;
    // private double sldrkOutputMax = 0.3;

    // private boolean sliderInitialized;

    public CoralSubsystem() {
        coralAngMtr = new SparkFlex(CoralConstants.CORAL_ROTATE_MOTOR_PORT, MotorType.kBrushless);
        // coralSldrMtr = new SparkMax(CoralConstants.CORAL_SLIDER_MOTOR_PORT, MotorType.kBrushless);
        outtakeMtrLdr = new SparkFlex(OuttakeConstants.OUTTAKE_LDR_PORT, MotorType.kBrushless);
        outtakeMtrFlw = new SparkFlex(OuttakeConstants.OUTTAKE_FLW_PORT, MotorType.kBrushless);
        // pusherServo = new Servo(CoralConstants.SERVO_PORT);
        coralAngMtrCfg = new SparkFlexConfig();
        //coralSldrMtrCfg = new SparkMaxConfig();
        outtakeMtrLdrCfg = new SparkFlexConfig();
        outtakeMtrFlwCfg = new SparkFlexConfig();

        coralSensor = new DigitalInput(CoralConstants.BEAM_BREAK_SENSOR_PORT);


        // coralLimitSwitch = coralSldrMtr.getForwardLimitSwitch();
        // encCfg = new AbsoluteEncoderConfig();
        // rotateMtrSftLmtCfg = new SoftLimitConfig();

        coralAngPID = coralAngMtr.getClosedLoopController();
        coralAngEnc = coralAngMtr.getAbsoluteEncoder();
        outtakeMtrLdrEnc = outtakeMtrLdr.getEncoder();
        outtakeMtrFlwEnc = outtakeMtrFlw.getEncoder();
        // coralSldrPID = coralSldrMtr.getClosedLoopController();
        // coralSldrEnc = coralSldrMtr.getEncoder();

        coralAngMtrCfg
            .inverted(false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake);
        coralAngMtrCfg
            .absoluteEncoder
                .positionConversionFactor(360);
        coralAngMtrCfg
            .softLimit
                .forwardSoftLimit(200.0) //swapped
                .reverseSoftLimit(60.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        coralAngMtrCfg
            .closedLoop
                .pidf(angkP, angkI, angkD, angkFF)
                .outputRange(angOutputMin, angOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
                // .maxMotion
                //     .allowedClosedLoopError(2.0);   
                //     .maxAcceleration(kMaxAccel)
                //     .maxVelocity(kMaxRPM)
                //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        coralAngMtr.configure(coralAngMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        // coralSldrMtrCfg
        //     .inverted(true)
        //     .voltageCompensation(12.0)
        //     .smartCurrentLimit(40)
        //     .idleMode(IdleMode.kBrake);
        // coralSldrMtrCfg
        //     .encoder
        //         .positionConversionFactor(360);//TO DO change to inches
        // coralSldrMtrCfg
        //     .softLimit
        //         .forwardSoftLimit(150.0) 
        //         .reverseSoftLimit(290.0);
        // coralSldrMtrCfg
        //     .limitSwitch
        //         .forwardLimitSwitchEnabled(true);
        // coralSldrMtrCfg
        //     .closedLoop
        //         .pidf(sldrkP, sldrkI, sldrkD, sldrkFF)
        //         .outputRange(sldrkOutputMin, sldrkOutputMax)
        //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        //         // .maxMotion
        //         //     .allowedClosedLoopError(2.0);   
        //         //     .maxAcceleration(kMaxAccel)
        //         //     .maxVelocity(kMaxRPM)
        //         //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        // coralSldrMtr.configure(coralSldrMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // coralSldrMtrCfg
        //     .inverted(true)
        //     .voltageCompensation(12.0)
        //     .smartCurrentLimit(40)
        //     .idleMode(IdleMode.kBrake);
        // coralSldrMtrCfg
        //     .encoder
        //         .positionConversionFactor(360);//TO DO change to inches
        // coralSldrMtrCfg
        //     .softLimit
        //         .forwardSoftLimit(150.0) 
        //         .reverseSoftLimit(290.0);
        // coralSldrMtrCfg
        //     .limitSwitch
        //         .forwardLimitSwitchEnabled(true);
        // coralSldrMtrCfg
        //     .closedLoop
        //         .pidf(sldrkP, sldrkI, sldrkD, sldrkFF)
        //         .outputRange(sldrkOutputMin, sldrkOutputMax)
        //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        //         // .maxMotion
        //         //     .allowedClosedLoopError(2.0);   
        //         //     .maxAcceleration(kMaxAccel)
        //         //     .maxVelocity(kMaxRPM)
        //         //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        // coralSldrMtr.configure(coralSldrMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
        outtakeMtrLdrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
      //  coralIndexMtrCfg
          //  .closedLoop
          //      .pid(indexkP, indexkI, indexkD)
      //          .outputRange(sldrkOutputMin, sldrkOutputMax)
       //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        outtakeMtrLdr.configure(outtakeMtrLdrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        outtakeMtrFlwCfg
            .follow(outtakeMtrLdr, true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);

        outtakeMtrFlw.configure(outtakeMtrFlwCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           
        // Add motors to the simulation
        if (Robot.isSimulation()) {
            coralAngMtrSim = new SparkFlexSim(coralAngMtr, DCMotor.getNEO(1));
            coralAngEncSim = new SparkAbsoluteEncoderSim(coralAngMtr);
            coralAngMtrSim.setPosition(190);
            coralAngEncSim.setPosition(190);
            coralAngMtrSim.setVelocity(0);
            coralAngEncSim.setVelocity(0);
            // coralSldrMtrSim = new SparkMaxSim(coralSldrMtr, DCMotor.getNEO(1));
            // coralSldrEncSim = new SparkRelativeEncoderSim(coralSldrMtr);
            outtakeMtrLdrSim = new SparkFlexSim(outtakeMtrLdr, DCMotor.getNEO(1));
            outtakeMtrLdrEncSim = new SparkRelativeEncoderSim(outtakeMtrLdr);
            outtakeMtrFlwSim = new SparkFlexSim(outtakeMtrLdr, DCMotor.getNEO(1));
            outtakeMtrFlwEncSim = new SparkRelativeEncoderSim(outtakeMtrLdr);
        }
    }
    
    // public void runIntakeCmd(double speed) {
    //     coralIndexMtr.set(speed);
    // }

    /**
     * Creates a FunctionalCommand to initialize the slider mechanism.
     * 
     * The command performs the following actions:
     * 1. Initializes the sliderInitialized flag to false.
     * 2. Continuously checks the state of the limit switch:
     *    - If the limit switch is not triggered, the coralSliderMtr motor is set to move the slider.
     *    - If the limit switch is triggered, the coralSliderMtr motor is stopped and the sliderInitialized flag is set to true.
     * 3. Stops the coralSliderMtr motor if the command is interrupted.
     * 4. Ends the command when the sliderInitialized flag is true.
     * 
     * @return A new FunctionalCommand instance for initializing the slider.
     */
    // public FunctionalCommand SliderInitCmd() {
    //     return new FunctionalCommand(() -> sliderInitialized = false,
    //                                  () -> {if(!coralLimitSwitch.isPressed()) {
    //                                             coralSldrMtr.set(-.125);
    //                                             //I think this can just be an else statement
    //                                         } else if(coralLimitSwitch.isPressed()) {
    //                                             coralSldrMtr.set(0);
    //                                             coralIndexEnc.setPosition(0);
    //                                             sliderInitialized = true;
    //                                         }},
    //                                  interrupted ->   coralSldrMtr.set(0),
    //                                  () -> sliderInitialized,
    //                                  this);
    // }

    /**
     * Creates a new FunctionalCommand for the intake mechanism.
     * 
     * The command performs the following actions:
     * - Initializes with an empty lambda function.
     * - Executes the intake mechanism with a speed of 0.1.
     * - Ends the intake mechanism by stopping it (setting speed to 0) when interrupted.
     * - Checks if the beam break sensor is triggered to determine if the command is finished.
     * 
     * @return A new instance of FunctionalCommand for controlling the intake mechanism.
     */
    public FunctionalCommand IntakeCmd() {
        return new FunctionalCommand(() ->{},
                                     () -> outtakeMtrLdr.set(-0.06),
                                     interrupted -> outtakeMtrLdr.set(0),
                                     () -> coralSensor.get() == true,
                                     this);
    }

    public FunctionalCommand OuttakeCmd() {
        return new FunctionalCommand(() ->{},
                                     () -> outtakeMtrLdr.set(-0.15),
                                     interrupted -> outtakeMtrLdr.set(0),
                                     () -> coralSensor.get() == false,
                                     this);
    }

    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
            () -> {},
            () -> setRotateAngle(pos), interrupted -> {},
            () -> (Math.abs(pos - coralAngEnc.getPosition()) <= 5.0),
            this);
    }

    
    // public FunctionalCommand rotateCMD() {
    //     return new FunctionalCommand(() ->{},
    //                                  () -> coralAngMtr.set(0.05),
    //                                  interrupted -> coralAngMtr.set(0.05),
    //                                  () -> !coralSensor.get(),
    //                                  this);
    // }

    // public FunctionalCommand setSliderPosition(double pos) {
    //     return new FunctionalCommand(() -> {},
    //         () -> coralSldrPID.setReference(pos, SparkMax.ControlType.kPosition),
    //         interrupted -> {},
    //         () -> (Math.abs(pos - coralSldrEnc.getPosition()) <= 5.0) && (Math.abs(coralSldrEnc.getVelocity()) <= 60.0),
    //         this);
    // }

    
    public void setRotateAngle(double angle) {
        coralAngPID.setReference(angle, SparkMax.ControlType.kPosition);
        // if (Robot.isSimulation()) {
        //     coralRotatePID.setReference(angle, SparkMax.ControlType.kPosition);
        // }
    }

    // public void setSliderPosition(double position) {
    //     coralSldrPID.setReference(position, SparkMax.ControlType.kPosition);
    //     // if (Robot.isSimulation()) {
    //     //     coralSliderPID.setReference(position, SparkMax.ControlType.kPosition);
    //     // }
    // }


    // public void extend() {
    //     pusherServo.set(1);
    // }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
        coralAngEncSim.setPosition(coralAngMtrSim.getPosition());
        coralAngMtrSim.iterate(coralAngEncSim.getPosition(), coralAngMtrSim.getBusVoltage(),.005);

        // coralSldrEncSim.setPosition(coralSldrMtrSim.getPosition());
        // coralSldrMtrSim.iterate(coralSldrEncSim.getPosition(), coralSldrMtrSim.getBusVoltage(),.005);
        
        outtakeMtrLdrEncSim.setPosition(outtakeMtrLdrSim.getPosition());
        outtakeMtrFlwEncSim.setPosition(outtakeMtrFlwSim.getPosition());
        outtakeMtrLdrSim.iterate(outtakeMtrLdrEncSim.getPosition(), outtakeMtrLdrSim.getBusVoltage(), .005);
        outtakeMtrFlwSim.iterate(outtakeMtrLdrEncSim.getPosition(), outtakeMtrFlwSim.getBusVoltage(), .005);
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Coarl Arm Position", coralAngEncSim.getPosition());
        // SmartDashboard.putNumber("Coral Slider Position", coralSldrEncSim.getPosition());
        SmartDashboard.putNumber("Outtake Speed", outtakeMtrLdrSim.getVelocity());
    } else {
        SmartDashboard.putNumber("Coral Arm Position", coralAngEnc.getPosition());
        SmartDashboard.putNumber("Coral Arm Speed", coralAngEnc.getVelocity());
        // SmartDashboard.putNumber("Coral Slider Position", coralSldrEnc.getPosition());
        SmartDashboard.putNumber("Outtake Speed", outtakeMtrLdrEnc.getVelocity());

        SmartDashboard.putBoolean("CoralSensor", coralSensor.get());

        // double distance = canrange.getDistance().getValueAsDouble();
        // close = distance < .43 && distance > .35;
        // SmartDashboard.putBoolean("canrange", close);
        //SmartDashboard.putNumber("canrange distance", distance);
    }

    }
}
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
import frc.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;                                                           
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class CoralSubsystem extends SubsystemBase {

    private SparkFlex armAngMtr;
    public AbsoluteEncoder armAngEnc;
    public SparkClosedLoopController  armAngPID;
    private SparkMax coralSldrMtr;
    public RelativeEncoder coralSldrEnc;
    public SparkFlex indexMtrLdr;
    public SparkFlex indexMtrFlw;
    public RelativeEncoder indexMtrLdrEnc;
    public RelativeEncoder indexMtrFlwEnc;
    public SparkClosedLoopController coralSldrPID;
    private SparkFlexSim armAngMtrSim;
    private SparkMaxSim coralSldrMtrSim;
    private SparkFlexSim indexMtrLdrSim;
    private SparkFlexSim indexMtrFlwSim;
    private SparkAbsoluteEncoderSim armAngEncSim; 
    private SparkRelativeEncoderSim indexMtrLdrEncSim;
    private SparkRelativeEncoderSim indexMtrFlwEncSim;
    private SparkRelativeEncoderSim coralSldrEncSim;
    private SparkFlexConfig armAngMtrCfg;
    private SparkMaxConfig coralSldrMtrCfg;
    private SparkFlexConfig indexMtrLdrCfg;
    private SparkFlexConfig indexMtrFlwCfg;
    private DigitalInput coralSensor;
    public SparkLimitSwitch armSliderLimitSwitch;
    private boolean sliderInitialized;
    
    private double angkP = 0.010, angkI = 0.0, angkD = 0.15;//p was 0.002
    private double angkFF = 0.0; //0.0075
    private double angOutputMin = -1.0;
    private double angOutputMax = 1.0;
    public boolean close;

    private double sldrkP = 1.25;
    private double sldrkOutputMin = -0.75;
    private double sldrkOutputMax = 0.75;

    // private boolean sliderInitialized;

    public CoralSubsystem() {
        armAngMtr = new SparkFlex(CoralConstants.CORAL_ROTATE_MOTOR_PORT, MotorType.kBrushless);
        coralSldrMtr = new SparkMax(CoralConstants.CORAL_SLIDER_MOTOR_PORT, MotorType.kBrushless);
        indexMtrLdr = new SparkFlex(OuttakeConstants.INDEX_LDR_PORT, MotorType.kBrushless);
        indexMtrFlw = new SparkFlex(OuttakeConstants.INDEX_FLW_PORT, MotorType.kBrushless);
        armAngMtrCfg = new SparkFlexConfig();
        coralSldrMtrCfg = new SparkMaxConfig();
        indexMtrLdrCfg = new SparkFlexConfig();
        indexMtrFlwCfg = new SparkFlexConfig();

        coralSensor = new DigitalInput(CoralConstants.BEAM_BREAK_SENSOR_PORT);

        armAngPID = armAngMtr.getClosedLoopController();
        armAngEnc = armAngMtr.getAbsoluteEncoder();
        indexMtrLdrEnc = indexMtrLdr.getEncoder();
        indexMtrFlwEnc = indexMtrFlw.getEncoder();
        coralSldrPID = coralSldrMtr.getClosedLoopController();
        coralSldrEnc = coralSldrMtr.getEncoder();

        armSliderLimitSwitch = coralSldrMtr.getForwardLimitSwitch();

        armAngMtrCfg
            .inverted(false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);
        armAngMtrCfg
            .absoluteEncoder
                .positionConversionFactor(360)
                .inverted(false)
                .zeroOffset(0.34722222);
        armAngMtrCfg
            .softLimit
                .forwardSoftLimit(200.0)
                .reverseSoftLimit(60.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        armAngMtrCfg
            .closedLoop
                .pid(angkP, angkI, angkD)
                .outputRange(angOutputMin, angOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armAngMtr.configure(armAngMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        coralSldrMtrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kCoast);
        coralSldrMtrCfg
            .encoder
                .positionConversionFactor(0.13352);
        coralSldrMtrCfg
            .softLimit
                .reverseSoftLimit(-12.0)
                .reverseSoftLimitEnabled(true);
        coralSldrMtrCfg
            .limitSwitch
                .forwardLimitSwitchType(Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true);
        coralSldrMtrCfg
            .closedLoop
                .p(sldrkP)
                .outputRange(sldrkOutputMin, sldrkOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        coralSldrMtr.configure(coralSldrMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        indexMtrLdrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake);
        indexMtrLdr.configure(indexMtrLdrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        indexMtrFlwCfg
            .follow(indexMtrLdr, true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kBrake);

        indexMtrFlw.configure(indexMtrFlwCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           
        // Add motors to the simulation
        if (Robot.isSimulation()) {
            armAngMtrSim = new SparkFlexSim(armAngMtr, DCMotor.getNEO(1));
            armAngEncSim = new SparkAbsoluteEncoderSim(armAngMtr);
            armAngMtrSim.setPosition(190);
            armAngEncSim.setPosition(190);
            armAngMtrSim.setVelocity(0);
            armAngEncSim.setVelocity(0);
            coralSldrMtrSim = new SparkMaxSim(coralSldrMtr, DCMotor.getNEO(1));
            coralSldrEncSim = new SparkRelativeEncoderSim(coralSldrMtr);
            indexMtrLdrSim = new SparkFlexSim(indexMtrLdr, DCMotor.getNEO(1));
            indexMtrLdrEncSim = new SparkRelativeEncoderSim(indexMtrLdr);
            indexMtrFlwSim = new SparkFlexSim(indexMtrLdr, DCMotor.getNEO(1));
            indexMtrFlwEncSim = new SparkRelativeEncoderSim(indexMtrLdr);
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
    public FunctionalCommand SliderInitCmd() {
        return new FunctionalCommand(() -> sliderInitialized = false,
                                     () -> {if(!armSliderLimitSwitch.isPressed()){ 
                                                coralSldrMtr.set(.175);}
                                            else if(armSliderLimitSwitch.isPressed()) {
                                                coralSldrMtr.set(0);
                                                coralSldrEnc.setPosition(0);
                                                sliderInitialized = true;
                                            }},
                                     interrupted ->   coralSldrMtr.set(0),
                                     () -> sliderInitialized,
                                     this);
    }

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
                                     () -> indexMtrLdr.set(-0.06),
                                     interrupted -> indexMtrLdr.set(0),
                                     () -> coralSensor.get() == true,
                                     this);
    }

    public FunctionalCommand OuttakeCmd() {
        return new FunctionalCommand(() ->{},
                                     () -> indexMtrLdr.set(-0.075),
                                     interrupted -> indexMtrLdr.set(0),
                                     () -> coralSensor.get() == false,
                                     this);
    }

    // public FunctionalCommand algaeOuttakeCmd() {
    //     return new FunctionalCommand(() ->{},
    //                                  () -> indexMtrLdr.set(-0.06),
    //                                  interrupted -> indexMtrLdr.set(0),
    //                                  () -> {},
    //                                  this);
    // }

    public Command algaeOuttakeCmd() {
        return this.runEnd(
            () -> {
                // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
                indexMtrLdr.set(0.50);
            }, ()-> {
              indexMtrLdr.set(0.25);
            }
          );
      }

      public Command algaeIntakeCmd() {
        return this.runEnd(
            () -> {
                // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
                indexMtrLdr.set(-0.75);
            }, ()-> {
                indexMtrLdr.set(0.0);
            }
          );
      }

    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
            () -> {},
            () -> setRotateAngle(pos), interrupted -> {},
            () -> (Math.abs(pos - armAngEnc.getPosition()) <= 5.0),
            this);
    }

    public FunctionalCommand setSliderPositionCmd(double pos) {
        return new FunctionalCommand(() -> {},
            () -> coralSldrPID.setReference(pos, SparkMax.ControlType.kPosition),
            interrupted -> {},
            () -> (Math.abs(pos - coralSldrEnc.getPosition()) <= 0.05),
            this);
    }

    public void sliderManual(double speed) {
        coralSldrMtr.set(speed);
    }

    public Command sliderManualCmd(double speed) {
        return this.runEnd(() -> {
            sliderManual(speed);
        }, () -> {coralSldrMtr.set(0.0);
        }
        );    
    }

    
    public void setRotateAngle(double angle) {
        armAngPID.setReference(angle, SparkMax.ControlType.kPosition);
        
        // From Minibot
        // This is an arbitrary feedforward value that is multiplied by the positon of the arm to account
        // for the reduction in force needed to hold the arm vertical instead of hortizontal.  The .abs
        // ensures the value is always positive.  The .cos function uses radians instead of degrees,
        // so the .toRadians converts from degrees to radians.
        // Use this to add a feed forward value to the arm to hold it horizontal - test test test!
        // Increase angkFF with the arm horizontal until just before it starts to drift upward
        armAngPID.setReference(angle,
                               SparkMax.ControlType.kPosition,
                               ClosedLoopSlot.kSlot0,
                               angkFF * Math.abs
                               (Math.cos
                               (Math.toRadians(angle - 115))),
                               ArbFFUnits.kPercentOut);

        // if (Robot.isSimulation()) {
        //     coralRotatePID.setReference(angle, SparkMax.ControlType.kPosition);
        // }
    }

    public void setSliderPosition(double position) {
        coralSldrPID.setReference(position, SparkMax.ControlType.kPosition);
        // if (Robot.isSimulation()) {
        //     coralSliderPID.setReference(position, SparkMax.ControlType.kPosition);
        // }
    }


    // public void extend() {
    //     pusherServo.set(1);
    // }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
        armAngEncSim.setPosition(armAngMtrSim.getPosition());
        armAngMtrSim.iterate(armAngEncSim.getPosition(), armAngMtrSim.getBusVoltage(),.005);

        coralSldrEncSim.setPosition(coralSldrMtrSim.getPosition());
        coralSldrMtrSim.iterate(coralSldrEncSim.getPosition(), coralSldrMtrSim.getBusVoltage(),.005);
        
        indexMtrLdrEncSim.setPosition(indexMtrLdrSim.getPosition());
        indexMtrFlwEncSim.setPosition(indexMtrFlwSim.getPosition());
        indexMtrLdrSim.iterate(indexMtrLdrEncSim.getPosition(), indexMtrLdrSim.getBusVoltage(), .005);
        indexMtrFlwSim.iterate(indexMtrLdrEncSim.getPosition(), indexMtrFlwSim.getBusVoltage(), .005);
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Coarl Arm Position", armAngEncSim.getPosition());
        SmartDashboard.putNumber("Coral Slider Position", coralSldrEncSim.getPosition());
        SmartDashboard.putNumber("Outtake Speed", indexMtrLdrSim.getVelocity());
    } else {
        SmartDashboard.putNumber("Coral Arm Position", armAngEnc.getPosition());
        SmartDashboard.putNumber("Coral Arm Speed", armAngEnc.getVelocity());
        SmartDashboard.putNumber("Coral Slider Position", coralSldrEnc.getPosition());
        SmartDashboard.putNumber("Outtake Speed", indexMtrLdrEnc.getVelocity());

        SmartDashboard.putBoolean("CoralSensor", coralSensor.get());

        SmartDashboard.putBoolean("Slider Switch", armSliderLimitSwitch.isPressed());

        // double distance = canrange.getDistance().getValueAsDouble();
        // close = distance < .43 && distance > .35;
        // SmartDashboard.putBoolean("canrange", close);
        //SmartDashboard.putNumber("canrange distance", distance);
    }

    }
}
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

public class RotateSubsystem extends SubsystemBase {

    private SparkFlex armAngMtr;
    public AbsoluteEncoder armAngEnc;
    public SparkClosedLoopController armAngPID;
    private SparkFlexSim armAngMtrSim;
    private SparkAbsoluteEncoderSim armAngEncSim;
    private SparkFlexConfig armAngMtrCfg;

    private double angkP = 0.010, angkI = 0.0, angkD = 0.15;// p was 0.002
    private double angkFF = 0.0; // 0.0075
    private double angOutputMin = -1.0;
    private double angOutputMax = 1.0;
    public boolean close;

    // private boolean sliderInitialized;

    public RotateSubsystem() {
        armAngMtr = new SparkFlex(CoralConstants.CORAL_ROTATE_MOTOR_PORT, MotorType.kBrushless);
        armAngMtrCfg = new SparkFlexConfig();

        armAngPID = armAngMtr.getClosedLoopController();
        armAngEnc = armAngMtr.getAbsoluteEncoder();

        armAngMtrCfg
                .inverted(false)
                .voltageCompensation(12.0)
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kCoast);
        armAngMtrCfg.absoluteEncoder
                .positionConversionFactor(360)
                .inverted(false)
                .zeroOffset(0.34722222);
        armAngMtrCfg.softLimit
                .forwardSoftLimit(200.0)
                .reverseSoftLimit(60.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        armAngMtrCfg.closedLoop
                .pid(angkP, angkI, angkD)
                .outputRange(angOutputMin, angOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armAngMtr.configure(armAngMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Add motors to the simulation
        if (Robot.isSimulation()) {
            armAngMtrSim = new SparkFlexSim(armAngMtr, DCMotor.getNEO(1));
            armAngEncSim = new SparkAbsoluteEncoderSim(armAngMtr);
            armAngMtrSim.setPosition(190);
            armAngEncSim.setPosition(190);
            armAngMtrSim.setVelocity(0);
            armAngEncSim.setVelocity(0);
        }
    }

    // public void runIntakeCmd(double speed) {
    // coralIndexMtr.set(speed);
    // }

    /**
     * Creates a FunctionalCommand to initialize the slider mechanism.
     * 
     * The command performs the following actions:
     * 1. Initializes the sliderInitialized flag to false.
     * 2. Continuously checks the state of the limit switch:
     * - If the limit switch is not triggered, the coralSliderMtr motor is set to
     * move the slider.
     * - If the limit switch is triggered, the coralSliderMtr motor is stopped and
     * the sliderInitialized flag is set to true.
     * 3. Stops the coralSliderMtr motor if the command is interrupted.
     * 4. Ends the command when the sliderInitialized flag is true.
     * 
     * @return A new FunctionalCommand instance for initializing the slider.
     */
    

    /**
     * Creates a new FunctionalCommand for the intake mechanism.
     * 
     * The command performs the following actions:
     * - Initializes with an empty lambda function.
     * - Executes the intake mechanism with a speed of 0.1.
     * - Ends the intake mechanism by stopping it (setting speed to 0) when
     * interrupted.
     * - Checks if the beam break sensor is triggered to determine if the command is
     * finished.
     * 
     * @return A new instance of FunctionalCommand for controlling the intake
     *         mechanism.
     */

    // public FunctionalCommand algaeOuttakeCmd() {
    // return new FunctionalCommand(() ->{},
    // () -> indexMtrLdr.set(-0.06),
    // interrupted -> indexMtrLdr.set(0),
    // () -> {},
    // this);
    // }

    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setRotateAngle(pos), interrupted -> {
                },
                () -> (Math.abs(pos - armAngEnc.getPosition()) <= 5.0),
                this);
    }

    public void setRotateAngle(double angle) {
        armAngPID.setReference(angle, SparkMax.ControlType.kPosition);

        // From Minibot
        // This is an arbitrary feedforward value that is multiplied by the positon of
        // the arm to account
        // for the reduction in force needed to hold the arm vertical instead of
        // hortizontal. The .abs
        // ensures the value is always positive. The .cos function uses radians instead
        // of degrees,
        // so the .toRadians converts from degrees to radians.
        // Use this to add a feed forward value to the arm to hold it horizontal - test
        // test test!
        // Increase angkFF with the arm horizontal until just before it starts to drift
        // upward
        armAngPID.setReference(angle,
                SparkMax.ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                angkFF * Math.abs(Math.cos(Math.toRadians(angle - 115))),
                ArbFFUnits.kPercentOut);

        // if (Robot.isSimulation()) {
        // coralRotatePID.setReference(angle, SparkMax.ControlType.kPosition);
        // }
    }

    // public void extend() {
    // pusherServo.set(1);
    // }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        armAngEncSim.setPosition(armAngMtrSim.getPosition());
        armAngMtrSim.iterate(armAngEncSim.getPosition(), armAngMtrSim.getBusVoltage(), .005);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (Robot.isSimulation()) {
            SmartDashboard.putNumber("Coarl Arm Position", armAngEncSim.getPosition());
        } else {
            SmartDashboard.putNumber("Coral Arm Position", armAngEnc.getPosition());
            SmartDashboard.putNumber("Coral Arm Speed", armAngEnc.getVelocity());
            // double distance = canrange.getDistance().getValueAsDouble();
            // close = distance < .43 && distance > .35;
            // SmartDashboard.putBoolean("canrange", close);
            // SmartDashboard.putNumber("canrange distance", distance);
        }
    }
}
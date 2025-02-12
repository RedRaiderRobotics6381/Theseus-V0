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

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralConstants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;                                                             

public class CoralSubsystem extends SubsystemBase {

    private SparkMax coralRotateMtr;
    public AbsoluteEncoder coralRotateEncoder;
    public SparkClosedLoopController  coralRotatePID;
    private SparkMax coralSliderMtr;
    public RelativeEncoder coralSliderEncoder;
    public SparkMax coralHoldMtr;
    public SparkClosedLoopController coralSliderPID;
    private SparkMaxSim coralRotateMtrSim;
    private SparkMaxSim coralSliderMtrSim;
    private SparkMaxSim coralIntakeMtrSim;
    private SparkMaxSim coralHoldMtrSim;
    private SparkAbsoluteEncoderSim coralRotateEncoderSim; 
    private SparkRelativeEncoderSim coralSliderEncoderSim;                            
    private SparkMaxConfig coralRotateMtrCfg;
    private SparkMaxConfig coralSliderMtrCfg;
    private SparkMaxConfig coralHoldMtrCfg;
    // private AbsoluteEncoderConfig encCfg;
    // private SoftLimitConfig rotateMtrSftLmtCfg;.
    
    private double rtekP = 0.005, rtekI = 0.0, rtekD = 0.0;//p was 0.0005
    private double rtekFF = 0.0;
    private double rtekOutputMin = -0.3;
    private double rtekOutputMax = 0.3;

    private double sldrkP = 0.005, sldrkI = 0.0, sldrkD = 0.0;//p was 0.0005
    private double sldrkFF = 0.0;
    private double sldrkOutputMin = -0.3;
    private double sldrkOutputMax = 0.3;

    public CoralSubsystem() {
        coralRotateMtr = new SparkMax(CoralConstants.CORAL_ROTATE_MOTOR_PORT, MotorType.kBrushless);
        coralSliderMtr = new SparkMax(CoralConstants.CORAL_SLIDER_MOTOR_PORT, MotorType.kBrushless);
        coralHoldMtr = new SparkMax(CoralConstants.CORAL_HOLD_MOTOR_PORT, MotorType.kBrushless);
        coralRotateMtrCfg = new SparkMaxConfig();
        coralSliderMtrCfg = new SparkMaxConfig();
        coralHoldMtrCfg = new SparkMaxConfig();
        // encCfg = new AbsoluteEncoderConfig();
        // rotateMtrSftLmtCfg = new SoftLimitConfig();

        coralRotatePID = coralRotateMtr.getClosedLoopController();
        coralRotateEncoder = coralRotateMtr.getAbsoluteEncoder();
        coralSliderPID = coralSliderMtr.getClosedLoopController();
        coralSliderEncoder = coralSliderMtr.getEncoder();
 
        coralRotateMtrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        coralRotateMtrCfg
            .absoluteEncoder
                .positionConversionFactor(360);
        coralRotateMtrCfg
            .softLimit
                .forwardSoftLimit(150.0) 
                .reverseSoftLimit(290.0);
        coralRotateMtrCfg
            .closedLoop
                .pidf(rtekP, rtekI, rtekD, rtekFF)
                .outputRange(rtekOutputMin, rtekOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
                // .maxMotion
                //     .allowedClosedLoopError(2.0);   
                //     .maxAcceleration(kMaxAccel)
                //     .maxVelocity(kMaxRPM)
                //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        coralRotateMtr.configure(coralRotateMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        coralSliderMtrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        coralSliderMtrCfg
            .encoder
                .positionConversionFactor(360);//TO DO change to inches
        coralSliderMtrCfg
            .softLimit
                .forwardSoftLimit(150.0) 
                .reverseSoftLimit(290.0);
        coralSliderMtrCfg
            .closedLoop
                .pidf(sldrkP, sldrkI, sldrkD, sldrkFF)
                .outputRange(sldrkOutputMin, sldrkOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                // .maxMotion
                //     .allowedClosedLoopError(2.0);   
                //     .maxAcceleration(kMaxAccel)
                //     .maxVelocity(kMaxRPM)
                //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        coralSliderMtr.configure(coralSliderMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        
        coralHoldMtrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        coralSliderMtr.configure(coralHoldMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           
        // Add motors to the simulation
        if (Robot.isSimulation()) {
            coralRotateMtrSim = new SparkMaxSim(coralRotateMtr, DCMotor.getNEO(1));
            coralRotateEncoderSim = new SparkAbsoluteEncoderSim(coralRotateMtr);
            coralRotateMtrSim.setPosition(190);
            coralRotateEncoderSim.setPosition(190);
            coralRotateMtrSim.setVelocity(0);
            coralRotateEncoderSim.setVelocity(0);
            coralSliderMtrSim = new SparkMaxSim(coralSliderMtr, DCMotor.getNEO(1));
            coralSliderEncoderSim = new SparkRelativeEncoderSim(coralSliderMtr);
            coralHoldMtrSim = new SparkMaxSim(coralHoldMtr, DCMotor.getNEO(1));
        }
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void RotateAlgae(double pos) {
        coralRotatePID.setReference(pos, SparkFlex.ControlType.kPosition);
        if (Robot.isSimulation()) {
            coralRotateMtrSim.setPosition(pos);
            coralRotateEncoderSim.setPosition(pos);
        }
    }

    public FunctionalCommand RotateAlgaeCmd(double pos) {
        // return new FunctionalCommand(() -> {},
                                        // () -> RotateAlgae(pos),
                                        // interrupted -> {},
                                        // () -> Math.abs(pos - rotateEncoder.getPosition()) <= 2.0, this);
        return new FunctionalCommand(() -> {},
                                     () -> RotateAlgae(pos), interrupted -> {},
                                     () -> (Math.abs(pos - coralRotateEncoder.getPosition()) <= 5.0) && (Math.abs(coralRotateEncoder.getVelocity()) <= 60.0),
                                     this);
    }

    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
        // rotateEncoderSim.setPosition(rotateMotorSim.getPosition());
        // rotateMotorSim.iterate(rotateEncoderSim.getPosition(), rotateMotorSim.getBusVoltage(),.005);
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    // if (Robot.isSimulation()) {
    //     SmartDashboard.putNumber("Arm Position", rotateEncoderSim.getPosition());
    // } else {
    //     SmartDashboard.putNumber("Arm Position", rotateEncoder.getPosition());
    // }
    }
}
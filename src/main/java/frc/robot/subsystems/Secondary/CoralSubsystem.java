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
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;                                                             

public class CoralSubsystem extends SubsystemBase {

    private SparkFlex rotateMotor;
    public AbsoluteEncoder rotateEncoder;
    public SparkClosedLoopController  rotatePID;
    private SparkFlexSim rotateMotorSim;
    private SparkAbsoluteEncoderSim rotateEncoderSim;                              
    private SparkFlexConfig rotateMtrCfg;
    // private AbsoluteEncoderConfig encCfg;
    // private SoftLimitConfig rotateMtrSftLmtCfg;.
    
    private double kP = 0.005, kI = 0.0, kD = 0.0;//p was 0.0005
    private double kFF = 0.0;
    private double kOutputMin = -0.3;
    private double kOutputMax = 0.3;

    public CoralSubsystem() {
        rotateMotor = new SparkFlex(Constants.ArmConstants.ARM_MOTOR_PORT, MotorType.kBrushless);
        rotateMtrCfg = new SparkFlexConfig();
        // encCfg = new AbsoluteEncoderConfig();
        // rotateMtrSftLmtCfg = new SoftLimitConfig();

        rotatePID = rotateMotor.getClosedLoopController();

        rotateEncoder = rotateMotor.getAbsoluteEncoder();
 
        rotateMtrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake);
        rotateMtrCfg
            .absoluteEncoder
                .positionConversionFactor(360);
        rotateMtrCfg
            .softLimit
                .forwardSoftLimit(150.0) 
                .reverseSoftLimit(290.0);
        rotateMtrCfg
            .closedLoop
                .pidf(kP, kI, kD, kFF)
                .outputRange(kOutputMin, kOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
                // .maxMotion
                //     .allowedClosedLoopError(2.0);   
                //     .maxAcceleration(kMaxAccel)
                //     .maxVelocity(kMaxRPM)
                //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        rotateMotor.configure(rotateMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           
        // Add motors to the simulation
        if (Robot.isSimulation()) {
            rotateMotorSim = new SparkFlexSim(rotateMotor, DCMotor.getNEO(1));
            rotateEncoderSim = new SparkAbsoluteEncoderSim(rotateMotor);
            rotateMotorSim.setPosition(190);
            rotateEncoderSim.setPosition(190);
            rotateMotorSim.setVelocity(0);
            rotateEncoderSim.setVelocity(0);
        }
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void RotateAlgae(double pos) {
        rotatePID.setReference(pos, SparkFlex.ControlType.kPosition);
        if (Robot.isSimulation()) {
            rotateMotorSim.setPosition(pos);
            rotateEncoderSim.setPosition(pos);
        }
    }

    public FunctionalCommand RotateAlgaeCmd(double pos) {
        // return new FunctionalCommand(() -> {},
                                        // () -> RotateAlgae(pos),
                                        // interrupted -> {},
                                        // () -> Math.abs(pos - rotateEncoder.getPosition()) <= 2.0, this);
        return new FunctionalCommand(() -> {},
                                     () -> RotateAlgae(pos), interrupted -> {},
                                     () -> (Math.abs(pos - rotateEncoder.getPosition()) <= 5.0) && (Math.abs(rotateEncoder.getVelocity()) <= 60.0),
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
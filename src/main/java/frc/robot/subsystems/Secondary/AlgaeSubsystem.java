// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * This is the AlgaeSubsystem class. This class is used to control the arm that holds the algae.
 * The arm has three main systems to control.
 * 
 * The first is the rotation of the arm which is driven
 * by a neoVortex motor with a SparkFlex controller. We will refer to this as algaeRotate.
 * 
 * The second is the launch system which is driven by (2) neoVortex motors with SparkFlex controllers.
 * We will refer to this as algaeLaunch.
 * 
 * The third is the intake system which is driven by (2) neo550 motors with SparkMax controllers, 
 * we will refer to this as algaeFeed.
 * 
 * The arm rotation has a limit switch at the the extent of its rotation to provide a way to
 * initialize the arm to a known position. This switch will be connected to the digital input on
 * on the SparkFlex controller (so we don't need to run the limit switch wiring all the way down
 * the elevator arm). The arm rotation has a soft limit opposite the limt switch to prevent the arm
 * from rotating too far.
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

public class AlgaeSubsystem extends SubsystemBase {

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

    public AlgaeSubsystem() {
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
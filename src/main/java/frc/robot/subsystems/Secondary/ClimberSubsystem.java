// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

public class ClimberSubsystem extends SubsystemBase {
 
    private SparkFlex climbMotor;
    public AbsoluteEncoder climbEncoder;
    public SparkClosedLoopController  climbPID;
    private SparkFlexSim climbMotorSim;
    private SparkAbsoluteEncoderSim climbEncoderSim;                              
    private SparkFlexConfig climbMtrCfg;
    
    private double kP = 0.015, kI = 0.0, kD = 0.0;//p was 0.0005
    private double kFF = 0.0;
    private double kOutputMin = -0.75;
    private double kOutputMax = 0.75;

    public ClimberSubsystem() {
        climbMotor = new SparkFlex(Constants.ClimbConstants.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
        climbMtrCfg = new SparkFlexConfig();
        //encCfg = new AbsoluteEncoderConfig();
        // rotateMtrSftLmtCfg = new SoftLimitConfig();

        climbPID = climbMotor.getClosedLoopController();

        climbEncoder = climbMotor.getAbsoluteEncoder();
 
        climbMtrCfg
            .inverted(false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        climbMtrCfg
            .absoluteEncoder
                .positionConversionFactor(360.0)
                .inverted(true)
                .zeroOffset(0.5); // was 0.4
        climbMtrCfg
            .softLimit
                .forwardSoftLimit(350.0) 
                .reverseSoftLimit(20.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        climbMtrCfg
            .closedLoop
                .pidf(kP, kI, kD, kFF)
                .outputRange(kOutputMin, kOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
                // .maxMotion
                //     .allowedClosedLoopError(2.0);   
                //     .maxAcceleration(kMaxAccel)
                //     .maxVelocity(kMaxRPM)
                //     .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        climbMotor.configure(climbMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
           
        // Add motors to the simulation
        if (Robot.isSimulation()) {
            climbMotorSim = new SparkFlexSim(climbMotor, DCMotor.getNEO(1));
            climbEncoderSim = new SparkAbsoluteEncoderSim(climbMotor);
            climbMotorSim.setPosition(190);
            climbEncoderSim.setPosition(190);
            climbMotorSim.setVelocity(0);
            climbEncoderSim.setVelocity(0);
        }
    }
    
    // // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setClimbPosition(double pos) {
        climbPID.setReference(pos, SparkFlex.ControlType.kPosition);
        if (Robot.isSimulation()) {
            climbMotorSim.setPosition(pos);
            climbEncoderSim.setPosition(pos);
        }
    }


    public FunctionalCommand climbAndGetPaid(double pos) {
        return new FunctionalCommand(
                () -> {},
                () -> setClimbPosition(pos),
                interrupted -> {},
                () -> (Math.abs(pos - climbEncoder.getPosition()) <= 5.0),
                this);
    }

    

    // public FunctionalCommand climbAndGetPaid(double pos) {
    //     return new FunctionalCommand(() -> releaseClimber(true),
    //     () -> setClimbPosition(pos),
    //     interrupted -> releaseClimber(false),
    //     () -> (Math.abs(pos - climbEncoder.getPosition()) <= 2.0),
    //     this);
    // }


    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        if (Robot.isSimulation()) {
            // rotateEncoderSim.setPosition(rotateMotorSim.getPosition());
            // rotateMotorSim.iterate(rotateEncoderSim.getPosition(), rotateMotorSim.getBusVoltage(),.005);
        }
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Climber Position", climbEncoderSim.getPosition());
    } else {
        SmartDashboard.putNumber("Climber Current", climbMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber Position", climbEncoder.getPosition());
    }
    }
}


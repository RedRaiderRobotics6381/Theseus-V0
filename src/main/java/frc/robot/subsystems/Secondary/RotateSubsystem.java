// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Secondary;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class RotateSubsystem extends SubsystemBase {

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

    public RotateSubsystem() {
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
    public void setArm(double pos) {
        rotatePID.setReference(pos, SparkFlex.ControlType.kPosition);
        if (Robot.isSimulation()) {
            rotateMotorSim.setPosition(pos);
            rotateEncoderSim.setPosition(pos);
        }


    }

    public FunctionalCommand RotatePosCmd(double pos) {
        // return new FunctionalCommand(() -> {}, () -> setArm(pos), interrupted -> {}, () -> Math.abs(pos - rotateEncoder.getPosition()) <= 2.0, this);
        return new FunctionalCommand(() -> {},
                                     () -> setArm(pos), interrupted -> {},
                                     () -> (Math.abs(pos - rotateEncoder.getPosition()) <= 5.0) && (Math.abs(rotateEncoder.getVelocity()) <= 60.0),
                                     this);
    }

    // public Command ForwardCmd() {
    // return this.run(
    //     () -> {
    //         setArm(Constants.ArmConstants.CORAL_INTAKE_POS);
    //     });
    // }

    // public Command MiddleCmd() {
    // return this.run(
    //     () -> {
    //         setArm(Constants.ArmConstants.CORAL_MID_POS);
    //     });
    // }

    // public Command UpCmd() {
    //   return this.run(
    //       () -> {
    //           setArm(Constants.ArmConstants.CORAL_HIGH_POS);
    //       });
    //   }

    // public Command AlgaeIntakeCmd () {
    //     return this.run(
    //         () -> {
    //             setArm(Constants.ArmConstants.ALGAE_INTAKE_POS);
    //         }); 
    // }

    // public Command AlgaeStartCmd () {
    //     return this.run(
    //         () -> {
    //             setArm(Constants.ArmConstants.ALGAE_START_POS);
    //         });
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
        SmartDashboard.putNumber("Arm Position", rotateEncoderSim.getPosition());
    } else {
        SmartDashboard.putNumber("Arm Position", rotateEncoder.getPosition());
    }
    }
}
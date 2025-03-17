// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Secondary;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

public class ClimberSubsystem extends SubsystemBase {
 
    private Servo climberServo;
    private SparkMax climbMotor;
    public AbsoluteEncoder climbEncoder;
    public SparkClosedLoopController  climbPID;
    private SparkMaxSim climbMotorSim;
    private SparkAbsoluteEncoderSim climbEncoderSim;                              
    private SparkMaxConfig climbMtrCfg;
    
    private double kP = 0.025, kI = 0.0, kD = 0.0;//p was 0.0005
    private double kFF = 0.0;
    private double kOutputMin = -1.0;
    private double kOutputMax = 1.0;

    public ClimberSubsystem() {
        climberServo = new Servo(Constants.ClimbConstants.CLIMBER_SERVO_PORT);
        climbMotor = new SparkMax(Constants.ClimbConstants.CLIMBER_MOTOR_PORT, MotorType.kBrushless);
        climbMtrCfg = new SparkMaxConfig();
        //encCfg = new AbsoluteEncoderConfig();
        // rotateMtrSftLmtCfg = new SoftLimitConfig();

        climbPID = climbMotor.getClosedLoopController();

        climbEncoder = climbMotor.getAbsoluteEncoder();
 
        climbMtrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        climbMtrCfg
            .absoluteEncoder
                .positionConversionFactor(360.0)
                .inverted(true)
                .zeroOffset(0.7249160);
        climbMtrCfg
            .softLimit
                .forwardSoftLimit(320.0) 
                .reverseSoftLimit(50.0);
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
            climbMotorSim = new SparkMaxSim(climbMotor, DCMotor.getNEO(1));
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
    /**releases the climber
     * 
     * @param release true to release, false to lock
     * 
     */
    public void releaseClimber(boolean release) {
        if (release) {
            climberServo.setAngle(0); //TODO: confirm angle
        } else{
            climberServo.setAngle(25); //TODO: confirm angle
        }
    }

    public FunctionalCommand climbAndGetPaid(double pos) {
        return new FunctionalCommand(() ->releaseClimber(true),
        () -> setClimbPosition(pos),
        interrupted -> {
        if(Math.abs(100.0 - climbEncoder.getPosition()) <= 2.0){
            releaseClimber(false);}},
        () -> (Math.abs(pos - climbEncoder.getPosition()) <= 2.0),
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
        SmartDashboard.putNumber("Climber Position", climbEncoder.getPosition());

        // System.out.println("Climber Position " + climbEncoder.getPosition());
    }
    }
}

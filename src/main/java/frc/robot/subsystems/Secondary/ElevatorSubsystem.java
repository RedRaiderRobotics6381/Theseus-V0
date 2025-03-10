// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * The elevator subsystem is driven by 2 neo vortex motors with sparkflex controllers. They will turn in the same
 * direction (not inverted). The leader motor will be the motor on the right side of the robot, and the follower motor
 * will be the motor on the left side of the robot. We assign CANID to the leader motor as 14 and the follower motor as 15.
 * The elevator has a limit switch at the bottom to prevent the elevator from going too low, this limit
 * switch is connected to the digital input 9, and is used in a routine to set the elevator to the
 * bottom position.
 * The gearbox ratio is 25:1, the pitch diameter of the driven sprocket is 1.92".
 * 1.92*pi = 6.03185789489" circumference, divided by the gearbox ratio of 25 gives us a ratio of 0.24127431" per revolution.
 * Likely this factor will change as the elevator is built and tested. 
 */
package frc.robot.subsystems.Secondary;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
// import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
// import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase {

    public SparkFlex elevMtrLdr;
    public SparkFlex elevMtrFlw;
    private SparkFlexConfig ldrCfg;
    private SparkFlexConfig flwCfg;
    public RelativeEncoder elevEncLdr;
    public RelativeEncoder elevEncFlw;
    public SparkClosedLoopController  elevPIDLdr;
    public SparkClosedLoopController  elevPIDFlw;
    private SparkFlexSim elevMtrLdrSim;
    private SparkFlexSim elevMtrFlwSim;
    private SparkRelativeEncoderSim elevEncLdrSim;
    private SparkRelativeEncoderSim elevEncFlwSim;
    private double kP = 0.075; //start p = 0.0005
    private double kD = 0.075;
    private double kOutput = 1.0;
    private double kMaxRPM = 2500;
    private double kMaxAccel = 8000;
    public DigitalInput limitSwL;
    private boolean elevatorInitialized;
    // public DigitalInput limitSwR;
    

    public ElevatorSubsystem() {
        elevMtrLdr = new SparkFlex(ElevatorConstants.LEFT_ELEVATOR_MOTOR_PORT, MotorType.kBrushless);
        elevMtrFlw = new SparkFlex(ElevatorConstants.RIGHT_ELEVATOR_MOTOR_PORT, MotorType.kBrushless);

        limitSwL = new DigitalInput(9);

        ldrCfg = new SparkFlexConfig();
        flwCfg = new SparkFlexConfig();

        elevPIDLdr = elevMtrLdr.getClosedLoopController();
        elevPIDFlw = elevMtrFlw.getClosedLoopController();

        elevEncLdr = elevMtrLdr.getEncoder();
        elevEncFlw = elevMtrFlw.getEncoder();

        ldrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        ldrCfg
            .encoder
                .positionConversionFactor(0.225); //confirm conversion factor
        ldrCfg
            .softLimit
                .forwardSoftLimit(21.0) 
                .reverseSoftLimit(0.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        ldrCfg
            .limitSwitch
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(true);
        ldrCfg
            .closedLoop
                // .pidf(kLdrP, kLdrI, kLdrD, kLdrFF)
                .p(kP)
                .d(kD)
                .outputRange(-kOutput, kOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kMaxAccel)
                    .maxVelocity(kMaxRPM)
                    .allowedClosedLoopError(0.5)
                    .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        elevMtrLdr.configure(ldrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flwCfg
            .follow(elevMtrLdr, false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        // flwCfg
        //     .encoder
        //         .positionConversionFactor(.2); //confirm conversion factor
        // flwCfg
        //     .softLimit
        //         .forwardSoftLimit(16.5) 
        //         .reverseSoftLimit(-0.5); // -0.05
        // flwCfg
        //     .closedLoop
        //         // .pidf(kFlwP, kFlwI, kFlwD, kFlwFF)
        //         .p(kFlwP)
        //         .outputRange(-0.5, 0.5)
        //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //         .maxMotion
        //             .maxAcceleration(kFlwMaxAccel)
        //             .maxVelocity(kFlwMaxRPM)
        //             .allowedClosedLoopError(0.125);
        elevMtrFlw.configure(flwCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Add motors to the simulation
        if (Robot.isSimulation()) {
            elevMtrLdrSim = new SparkFlexSim(elevMtrLdr, DCMotor.getNEO(1));
            elevMtrFlwSim = new SparkFlexSim(elevMtrFlw, DCMotor.getNEO(1));
            elevEncLdrSim = new SparkRelativeEncoderSim(elevMtrLdr);
            elevEncFlwSim = new SparkRelativeEncoderSim(elevMtrFlw);
            elevMtrLdrSim.setPosition(0);
            elevMtrFlwSim.setPosition(0);
            elevEncLdrSim.setVelocity(0);
            elevEncFlwSim.setVelocity(0);

        }
    }
    
    // An accessor method to set the speed (technically the output percentage) of the launch wheel
    public void setElevatorHeight(double pos) {
        // leaderElevatorL.set(speed);
        elevPIDLdr.setReference(pos, SparkMax.ControlType.kMAXMotionPositionControl);
        if (Robot.isSimulation()) {
            // leaderElevatorSim.setVelocity(speed);
            // followerElevatorSim.setVelocity(speed);
            // if (!limitSwL.get()) {
            //     elevPIDLdr.setReference(pos, SparkMax.ControlType.kPosition);
            // }
            // else {
            //     elevMtrLdr.set(0);
            // }

            // elevPIDLdr.setReference(pos, SparkMax.ControlType.kPosition);
        }
    }

    public Command INIT_POSE() {
        return this.run(
            () -> {
                if (limitSwL.get()){
                    elevMtrLdr.set(.125);
                }
                else{
                    elevMtrLdr.set(0);
                    elevEncLdr.setPosition(0);
                    elevEncFlw.setPosition(0);
                }
            });
    }

    public FunctionalCommand ElevatorHeightCmd(double height) {
        return new FunctionalCommand(() -> {},
            () -> setElevatorHeight(height),
            interrupted -> {},
            () -> Math.abs(height - elevEncLdr.getPosition()) <= 0.5,
            this);
    }

    // public Command START_POSE() {
    //     return this.run(
    //         () -> {
    //             setElevatorHeight(ElevatorConstants.START_POSE);
    //         });
    // }

    // public Command REEF_LOW_POSE() {
    //     return this.run(
    //         () -> {
    //             setElevatorHeight(ElevatorConstants.REEF_LOW_POSE);
    //         });
    //     }

    // public Command REEF_MIDDLE_POSE() {
    //     return this.run(
    //         () -> {
    //             setElevatorHeight(ElevatorConstants.REEF_MIDDLE_POSE);
    //         });
    //     }

    // public Command REEF_HIGH_POSE() {
    //     return this.run(
    //         () -> {
    //             setElevatorHeight(ElevatorConstants.REEF_HIGH_POSE);
    //         });
    //     }

    public FunctionalCommand ElevatorInitCmd() {
        return new FunctionalCommand(() -> elevatorInitialized = false,
                                        () -> {if(limitSwL.get()){
                                                elevMtrLdr.set(-.125);
                                            } else if(!limitSwL.get()) {
                                                elevMtrLdr.set(0);
                                                elevEncLdr.setPosition(0);
                                                elevatorInitialized = true;
                                            }},
                                        interrupted ->   elevMtrLdr.set(0),
                                        () -> elevatorInitialized,
                                        this);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // if (Robot.isSimulation()) {
        //     elevEncLdrSim.setPosition(elevMtrLdrSim.getPosition());
        //     elevEncFlwSim.setPosition(elevMtrFlwSim.getPosition());
        //     elevMtrLdrSim.iterate(elevEncLdrSim.getPosition(), elevMtrLdrSim.getBusVoltage(),.005);
        //     elevMtrFlwSim.iterate(elevEncFlwSim.getPosition(), elevMtrFlwSim.getBusVoltage(),.005);
        // }
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Elevator Position", elevEncLdrSim.getPosition());
        // SmartDashboard.putNumber("Elevator Follower Speed (RPM)", elevEncFlwSim.getPosition());
    } else {
        SmartDashboard.putNumber("Elevator Position", elevEncLdr.getPosition());
        // SmartDashboard.putNumber("Elevator Follower Position", elevEncFlw.getPosition());
        SmartDashboard.putBoolean("Elevator Limit Switch", !limitSwL.get());

        SmartDashboard.putNumber("Elevator Current", elevMtrLdr.getOutputCurrent());

        SmartDashboard.putNumber("Elevator Speed", elevEncLdr.getVelocity());
       }
    }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * The elevator subsystem is driven by 2 neo vortex motors with sparkflex controllers. They will turn in the same
 * direction (not inverted). The leader motor will be the motor on the right side of the robot, and the follower motor
 * will be the motor on the left side of the robot. We will CANID the leader motor as 14 and the follower motor as 15.
 * The elevator has a limit switch at the bottom to prevent the elevator from going too low, thia limit
 * switch is connected to the digital input 0, and is used in a routine to set the elevator to the
 * bottom position.
 * The gearbox ratio is 60:7 or 8.571428571428571:1, the pitch diameter of the 22 tooth driven sprocket is 1.751".
 * 1.751*pi = 5.500929" circumference, divided by the gearbox ratio of 8.571428571428571 gives us a ration of 0.64177501" per revolution.
 * Likely this ratio will change as the elevator is built and tested. 
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
    private double kLdrP = 0.125, kLdrI = 0.0, kLdrD = 0.0; //start p = 0.0005
    private double kFlwP = 0.125, kFlwI = 0.0, kFlwD = 0.0;
    private double kLdrFF = 0.0005, kFlwFF = 0.0005;
    private double kLdrOutputMin = -0.25, kFlwOutputMin = -0.25;
    private double kLdrOutputMax = 0.25, kFlwOutputMax = 0.25;
    private double kLdrMaxRPM = 100, kFlwMaxRPM = 100;
    private double kLdrMaxAccel = 50, kFlwMaxAccel = 50;
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
                .positionConversionFactor(0.2); //confirm conversion factor
        ldrCfg
            .softLimit
                .forwardSoftLimit(16.5) 
                .reverseSoftLimit(-1.0);
        // ldrCfg
        //     .limitSwitch
        //     .reverseLimitSwitchType(Type.kNormallyClosed)
        //     .reverseLimitSwitchEnabled(true);
        ldrCfg
            .closedLoop
                .pidf(kLdrP, kLdrI, kLdrD, kLdrFF)
                .outputRange(kLdrOutputMin, kLdrOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kLdrMaxAccel)
                    .maxVelocity(kLdrMaxRPM)
                    .allowedClosedLoopError(0.125);
        elevMtrLdr.configure(ldrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        flwCfg
            .follow(elevMtrLdr, false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        flwCfg
            .encoder
                .positionConversionFactor(.2); //confirm conversion factor
        flwCfg
            .softLimit
                .forwardSoftLimit(16.5) 
                .reverseSoftLimit(-0.5); // -0.05
        flwCfg
            .closedLoop
                .pidf(kFlwP, kFlwI, kFlwD, kFlwFF)
                .outputRange(kFlwOutputMin, kFlwOutputMax)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .maxMotion
                    .maxAcceleration(kFlwMaxAccel)
                    .maxVelocity(kFlwMaxRPM)
                    .allowedClosedLoopError(0.125);
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
        elevPIDLdr.setReference(pos, SparkMax.ControlType.kPosition);
        if (Robot.isSimulation()) {
            // leaderElevatorSim.setVelocity(speed);
            // followerElevatorSim.setVelocity(speed);
            // if (!limitSwL.get()) {
            //     elevPIDLdr.setReference(pos, SparkMax.ControlType.kPosition);
            // }
            // else {
            //     elevMtrLdr.set(0);
            // }

            elevPIDLdr.setReference(pos, SparkMax.ControlType.kPosition);
        }
    }

    public Command INIT_POSE() {
        return this.run(
            () -> {
                if (!limitSwL.get()){
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
            () -> Math.abs(height - elevEncLdr.getPosition()) <= 0.125,
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
                                        () -> {if(!limitSwL.get()){
                                                elevMtrLdr.set(-.125);
                                            } else if(limitSwL.get()) {
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
        SmartDashboard.putBoolean("Elevator Limit Switch", limitSwL.get());
       }
    }
}
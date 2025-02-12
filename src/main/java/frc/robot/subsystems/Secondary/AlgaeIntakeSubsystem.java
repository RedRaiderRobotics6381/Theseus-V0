// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new PincherSubsystem. */
  private SparkFlex intakeMtrLdr;
  private SparkFlex intakeMtrFlw;
  private SparkMax indexerMtrLdr;
  private SparkMax indexerMtrFlw;
  private RelativeEncoder intakeEncLdr;
//   private RelativeEncoder intakeEncFlw;
  public SparkClosedLoopController  intakeLdrPID;
  public SparkClosedLoopController  intakeFlwPID;
  private SparkFlexSim intakeMtrLdrSim;
  private SparkFlexSim intakeMtrFlwSim;
  private SparkMaxSim indexerMtrLdrSim;
  private SparkMaxSim indexerMtrFlwSim;
  private SparkFlexConfig ldrCfg;
  private SparkFlexConfig flwCfg;
  private SparkMaxConfig indexerLdrCfg;
  private SparkMaxConfig indexerFlwCfg;
  private SparkRelativeEncoderSim intakeEncLdrSim;
  private SparkRelativeEncoderSim intakeEncFlwSim;
  // private double kLdrP = 0.0003, kLdrI = 0.0, kLdrD = 0.0;
  // private double kFlwP = 0.0003, kFlwI = 0.0, kFlwD = 0.0;
  // private double kLdrFF = 0.0005, kFlwFF = 0.0005;
  // private double kLdrOutputMin = -1.0, kFlwOutputMin = -1.0;
  // private double kLdrOutputMax = 1.0, kFlwOutputMax = 1.0;
  // private double kLdrMaxRPM = 5676, kFlwMaxRPM = 5676;
  // private double kLdrMaxAccel = 10000, kFlwMaxAccel = 10000;
  
  public AlgaeIntakeSubsystem() {
    intakeMtrLdr = new SparkFlex(AlgaeIntakeConstants.LEFT_INTAKE_MOTOR_PORT, MotorType.kBrushless);
    intakeMtrFlw = new SparkFlex(AlgaeIntakeConstants.RIGHT_INTAKE_MOTOR_PORT, MotorType.kBrushless);
    indexerMtrLdr = new SparkMax(AlgaeIntakeConstants.LEFT_INDEXER_MOTOR_PORT, MotorType.kBrushless);
    indexerMtrFlw = new SparkMax(AlgaeIntakeConstants.RIGHT_INDEXER_MOTOR_PORT, MotorType.kBrushless);
    
    ldrCfg = new SparkFlexConfig();
    flwCfg = new SparkFlexConfig();
    indexerLdrCfg = new SparkMaxConfig();
    indexerFlwCfg = new SparkMaxConfig();
    // SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();
    // SoftLimitConfig followerSoftLimit = new SoftLimitConfig();

    // intakeLdrPID = intakeMtrLdr.getClosedLoopController();
    // intakeFlwPID = intakeMtrFlw.getClosedLoopController();

    intakeEncLdr = intakeMtrLdr.getEncoder();
    // intakeEncFlw = intakeMtrFlw.getEncoder();

    ldrCfg
        .inverted(false)
        .voltageCompensation(12.0)
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);
        // .closedLoop
        //     .pid(kLdrP, kLdrI, kLdrD)
        //     .outputRange(kLdrOutputMin, kLdrOutputMax)
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .maxMotion
        //         .maxAcceleration(kLdrMaxAccel)
        //         .maxVelocity(kLdrMaxRPM);
    intakeMtrLdr.configure(ldrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flwCfg
        .follow(intakeMtrLdr, true)
        .voltageCompensation(12.0)
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);
        // .closedLoop
        //     .pid(kFlwP, kFlwI, kFlwD)
        //     .outputRange(kFlwOutputMin, kFlwOutputMax)
        //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .maxMotion
        //         .maxAcceleration(kFlwMaxAccel)
        //         .maxVelocity(kFlwMaxRPM);
    intakeMtrFlw.configure(flwCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    indexerLdrCfg
        .inverted(false)
        .voltageCompensation(12.0)
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);
    intakeMtrLdr.configure(ldrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    indexerFlwCfg
        .follow(indexerMtrLdr, true)
        .voltageCompensation(12.0)
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);
    intakeMtrLdr.configure(ldrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    if (Robot.isSimulation()) {
      intakeMtrLdrSim = new SparkFlexSim(intakeMtrLdr, DCMotor.getNEO(1));
      intakeMtrFlwSim = new SparkFlexSim(intakeMtrFlw, DCMotor.getNEO(1));
      indexerMtrLdrSim = new SparkMaxSim(indexerMtrLdr, DCMotor.getNEO(1));
      indexerMtrFlwSim = new SparkMaxSim(indexerMtrFlw, DCMotor.getNEO(1));
      intakeEncLdrSim = new SparkRelativeEncoderSim(intakeMtrLdr);
      intakeEncFlwSim = new SparkRelativeEncoderSim(intakeMtrFlw);
      // leaderIntakeSim.setVelocity(0);
      // followerIntakeSim.setVelocity(0);
      // leaderEncoderSim.setVelocity(0);
      // followerEncoderSim.setVelocity(0);
    }

  }

  @Override
  public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
      if (Robot.isSimulation()) {
          // leaderEncoderSim.setPosition(rotateMotorSim.getPosition());
          intakeMtrLdrSim.iterate(intakeEncLdrSim.getVelocity(), intakeMtrLdrSim.getBusVoltage(),.005);
          intakeMtrFlwSim.iterate(intakeEncFlwSim.getVelocity(), intakeMtrFlwSim.getBusVoltage(),.005);
          
      }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
        SmartDashboard.putNumber("Intake (%)", intakeEncLdrSim.getPosition());
        // SmartDashboard.putNumber("Intake Follower Speed (RPM)", intakeEncFlwSim.getPosition());
    } else {
        SmartDashboard.putNumber("Intake Speed (%)", intakeEncLdr.getPosition());
        // SmartDashboard.putNumber("Intake Follower Speed (RPM)", intakeEncFlw.getPosition());
    }
  }
  

  // public void runIntake(double speed) {
  //   intakeLdrPID.setReference(speed, SparkFlex.ControlType.kVelocity);
  // }

  // public Command IntakeCmd() {
  //   return this.runEnd(
  //       () -> {
  //           // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
  //           intakeMtrLdr.set(0.15);
  //       },
  //       () -> {
  //           // runIntake(Constants.IntakeConstants.HOLD_SPEED);
  //           intakeMtrLdr.set(0.1);
  //       }
  //     );
  // }

  public Command RunIntakeCmd(double speed) {
    return this.run(
        () -> {
            // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
            intakeMtrLdr.set(speed);
            indexerMtrLdr.set(speed);
        }
      );
  }

  public Command RunHoldCmd() {
    return this.run(
        () -> {
            // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
            indexerMtrLdr.set(0.1);
        }
      );
  }

  public Command RunOuttakeCmd() {
    return this.run(
        () -> {
            // runIntake(Constants.IntakeConstants.OUTTAKE_SPEED);
            intakeMtrLdr.set(-0.15);
            if (intakeMtrLdr.getEncoder().getVelocity() >= 500){
                indexerMtrLdr.set(-0.15);
            }
        }
      );
  }

    // public Command StopCmd() {
  //   return this.runOnce(
  //       () -> {
  //           runIntake(Constants.IntakeConstants.STOP_SPEED);
  //       }
  //     );
  // }

}






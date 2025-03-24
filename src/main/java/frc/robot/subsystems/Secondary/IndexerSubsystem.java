// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Robot;

public class IndexerSubsystem extends SubsystemBase {

  public SparkFlex indexMtrLdr;
  public SparkFlex indexMtrFlw;
  public RelativeEncoder indexMtrLdrEnc;
  public RelativeEncoder indexMtrFlwEnc;
  private SparkFlexSim indexMtrLdrSim;
  private SparkFlexSim indexMtrFlwSim;
  private SparkRelativeEncoderSim indexMtrLdrEncSim;
  private SparkRelativeEncoderSim indexMtrFlwEncSim;
  private SparkFlexConfig indexMtrLdrCfg;
  private SparkFlexConfig indexMtrFlwCfg;
  private DigitalInput coralSensor;

  public IndexerSubsystem() {

    indexMtrLdr = new SparkFlex(OuttakeConstants.INDEX_LDR_PORT, MotorType.kBrushless);
    indexMtrFlw = new SparkFlex(OuttakeConstants.INDEX_FLW_PORT, MotorType.kBrushless);
    indexMtrLdrCfg = new SparkFlexConfig();
    indexMtrFlwCfg = new SparkFlexConfig();
    indexMtrLdrEnc = indexMtrLdr.getEncoder();
    indexMtrFlwEnc = indexMtrFlw.getEncoder();
    coralSensor = new DigitalInput(CoralConstants.BEAM_BREAK_SENSOR_PORT);

    indexMtrLdrCfg
        .inverted(true)
        .voltageCompensation(12.0)
        .smartCurrentLimit(65)
        .idleMode(IdleMode.kBrake);
    indexMtrLdr.configure(indexMtrLdrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    indexMtrFlwCfg
        .follow(indexMtrLdr, true)
        .voltageCompensation(12.0)
        .smartCurrentLimit(65)
        .idleMode(IdleMode.kBrake);
    indexMtrFlw.configure(indexMtrFlwCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (Robot.isSimulation()) {
      indexMtrLdrSim = new SparkFlexSim(indexMtrLdr, DCMotor.getNEO(1));
      indexMtrLdrEncSim = new SparkRelativeEncoderSim(indexMtrLdr);
      indexMtrFlwSim = new SparkFlexSim(indexMtrLdr, DCMotor.getNEO(1));
      indexMtrFlwEncSim = new SparkRelativeEncoderSim(indexMtrLdr);
    }
  }

  public FunctionalCommand IntakeCmd() {
    return new FunctionalCommand(() -> {
    },
        () -> indexMtrLdr.set(-0.06),
        interrupted -> indexMtrLdr.set(0),
        () -> coralSensor.get() == true,
        this);
  }

  public StartEndCommand AccurateOuttake(){
    return new StartEndCommand(
      () -> indexMtrLdr.set(-0.05), 
      () -> indexMtrLdr.set(0), 
      this);
  }
  public StartEndCommand Retract(){
    return new StartEndCommand(
      () -> indexMtrLdr.set(0.075), 
      () -> indexMtrLdr.set(0), 
      this);
  }

  public FunctionalCommand OuttakeCmd() {
    return new FunctionalCommand(() -> {
    },
        () -> indexMtrLdr.set(-0.075),
        interrupted -> indexMtrLdr.set(0),
        () -> coralSensor.get() == false,
        this);
  }

  public Command algaeOuttakeCmd() {
    return this.runEnd(
        () -> {
          // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
          indexMtrLdr.set(0.50);
        }, () -> {
          indexMtrLdr.set(0.5);
        });
  }

  public Command algaeIntakeCmd() {
    return this.runEnd(
        () -> {
          // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
          indexMtrLdr.set(-1.0);
        }, () -> {
          indexMtrLdr.set(0.0);
        });
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    indexMtrLdrEncSim.setPosition(indexMtrLdrSim.getPosition());
    indexMtrFlwEncSim.setPosition(indexMtrFlwSim.getPosition());
    indexMtrLdrSim.iterate(indexMtrLdrEncSim.getPosition(), indexMtrLdrSim.getBusVoltage(), .005);
    indexMtrFlwSim.iterate(indexMtrLdrEncSim.getPosition(), indexMtrFlwSim.getBusVoltage(), .005);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      SmartDashboard.putNumber("Outtake Speed", indexMtrLdrSim.getVelocity());
    } else {
      SmartDashboard.putNumber("Outtake Speed", indexMtrLdrEnc.getVelocity());
      SmartDashboard.putBoolean("CoralSensor", coralSensor.get());
    }
  }
}

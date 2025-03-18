// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Secondary;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Robot;

public class SliderSubsystem extends SubsystemBase {
  private SparkMax coralSldrMtr;
  public RelativeEncoder coralSldrEnc;
  public SparkClosedLoopController coralSldrPID;
  private SparkMaxSim coralSldrMtrSim;
  private SparkRelativeEncoderSim coralSldrEncSim;
  private SparkMaxConfig coralSldrMtrCfg;
  public SparkLimitSwitch armSliderLimitSwitch;

  private boolean sliderInitialized;

  private double sldrkP = 1.25;
  private double sldrkOutputMin = -0.75;
  private double sldrkOutputMax = 0.75;

  public SliderSubsystem() {

    coralSldrMtr = new SparkMax(CoralConstants.CORAL_SLIDER_MOTOR_PORT, MotorType.kBrushless);
    coralSldrMtrCfg = new SparkMaxConfig();
    coralSldrPID = coralSldrMtr.getClosedLoopController();
    coralSldrEnc = coralSldrMtr.getEncoder();

    armSliderLimitSwitch = coralSldrMtr.getForwardLimitSwitch();

    coralSldrMtrCfg
        .inverted(true)
        .voltageCompensation(12.0)
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kCoast);
    coralSldrMtrCfg.encoder
        .positionConversionFactor(0.13352);
    coralSldrMtrCfg.softLimit
        .reverseSoftLimit(-12.0)
        .reverseSoftLimitEnabled(true);
    coralSldrMtrCfg.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true);
    coralSldrMtrCfg.closedLoop
        .p(sldrkP)
        .outputRange(sldrkOutputMin, sldrkOutputMax)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    coralSldrMtr.configure(coralSldrMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Add motors to the simulation
    if (Robot.isSimulation()) {
      coralSldrMtrSim = new SparkMaxSim(coralSldrMtr, DCMotor.getNEO(1));
      coralSldrEncSim = new SparkRelativeEncoderSim(coralSldrMtr);
    }

  }

  public FunctionalCommand SliderInitCmd() {
    return new FunctionalCommand(() -> sliderInitialized = false,
        () -> {
          if (!armSliderLimitSwitch.isPressed()) {
            coralSldrMtr.set(.175);
          } else if (armSliderLimitSwitch.isPressed()) {
            coralSldrMtr.set(0);
            coralSldrEnc.setPosition(0);
            sliderInitialized = true;
          }
        },
        interrupted -> coralSldrMtr.set(0),
        () -> sliderInitialized,
        this);
  }

  public FunctionalCommand setSliderPositionCmd(double pos) {
    return new FunctionalCommand(() -> {
    },
        () -> coralSldrPID.setReference(pos, SparkMax.ControlType.kPosition),
        interrupted -> {
        },
        () -> (Math.abs(pos - coralSldrEnc.getPosition()) <= 0.05),
        this);
  }

  public void sliderManual(double speed) {
    coralSldrMtr.set(speed);
  }

  public Command sliderManualCmd(double speed) {
    return this.runEnd(() -> {
      sliderManual(speed);
    }, () -> {
      coralSldrMtr.set(0.0);
    });
  }

  public void setSliderPosition(double position) {
    coralSldrPID.setReference(position, SparkMax.ControlType.kPosition);
    // if (Robot.isSimulation()) {
    // coralSliderPID.setReference(position, SparkMax.ControlType.kPosition);
    // }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    coralSldrEncSim.setPosition(coralSldrMtrSim.getPosition());
    coralSldrMtrSim.iterate(coralSldrEncSim.getPosition(), coralSldrMtrSim.getBusVoltage(), .005);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.isSimulation()) {
      SmartDashboard.putNumber("Coral Slider Position", coralSldrEncSim.getPosition());
    } else {
      SmartDashboard.putNumber("Coral Slider Position", coralSldrEnc.getPosition());
      SmartDashboard.putBoolean("Slider Switch", armSliderLimitSwitch.isPressed());

      // double distance = canrange.getDistance().getValueAsDouble();
      // close = distance < .43 && distance > .35;
      // SmartDashboard.putBoolean("canrange", close);
      // SmartDashboard.putNumber("canrange distance", distance);
    }
  }
}

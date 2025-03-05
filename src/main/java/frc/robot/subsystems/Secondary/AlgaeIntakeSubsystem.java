// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Secondary;

// import com.ctre.phoenix6.signals.MotorArrangementValue;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.sim.SparkFlexSim;
// import com.revrobotics.sim.SparkMaxSim;
// import com.revrobotics.sim.SparkRelativeEncoderSim;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkFlexConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj2.command.Command;
// // import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.*;
// import frc.robot.Robot;

// public class AlgaeIntakeSubsystem extends SubsystemBase {
//   /** Creates a new PincherSubsystem. */
//   private SparkFlex launcherMtrLdr;
//   private SparkFlex launcherMtrFlw;
//   private SparkMax feederMtrLdr;
//   //private SparkMax feederMtrFlw;
//   private RelativeEncoder launcherEncLdr;
//   private RelativeEncoder feederEncLdr;
// //   private RelativeEncoder intakeEncFlw;
//   public SparkClosedLoopController  launcherLdrPID;
//   public SparkClosedLoopController  launcherFlwPID;
//   private SparkFlexSim launcherMtrLdrSim;
//   private SparkFlexSim launcherMtrFlwSim;
//   private SparkMaxSim feederMtrLdrSim;
//   //private SparkMaxSim feederMtrFlwSim;
//   private SparkFlexConfig launcherLdrCfg;
//   private SparkFlexConfig LauncherFlwCfg;
//   private SparkMaxConfig feederLdrCfg;
//   //private SparkMaxConfig feederFlwCfg;
//   private SparkRelativeEncoderSim launcherEncLdrSim;
//   private SparkRelativeEncoderSim launcherEncFlwSim;
//   private SparkRelativeEncoderSim feederEncLdrSim;
//   //private SparkRelativeEncoderSim feederEncFlwSim;

//   // private double kLdrP = 0.0003, kLdrI = 0.0, kLdrD = 0.0;
//   // private double kFlwP = 0.0003, kFlwI = 0.0, kFlwD = 0.0;
//   // private double kLdrFF = 0.0005, kFlwFF = 0.0005;
//   // private double kLdrOutputMin = -1.0, kFlwOutputMin = -1.0;
//   // private double kLdrOutputMax = 1.0, kFlwOutputMax = 1.0;
//   // private double kLdrMaxRPM = 5676, kFlwMaxRPM = 5676;
//   // private double kLdrMaxAccel = 10000, kFlwMaxAccel = 10000;
  
//   public AlgaeIntakeSubsystem() {
//     launcherMtrLdr = new SparkFlex(AlgaeIntakeConstants.LAUNCHER_FOLLOWER_MOTOR_PORT, MotorType.kBrushless);
//     launcherMtrFlw = new SparkFlex(AlgaeIntakeConstants.LAUNCHER_LEADER_MOTOR_PORT, MotorType.kBrushless);
//     feederMtrLdr = new SparkMax(AlgaeIntakeConstants.FEEDER_LEADER_MOTOR_PORT, MotorType.kBrushless);
//     // feederMtrFlw = new SparkMax(AlgaeIntakeConstants.FEEDER_LEADER_MOTOR_PORT, MotorType.kBrushless);
    
//     launcherLdrCfg = new SparkFlexConfig();
//     LauncherFlwCfg = new SparkFlexConfig();
//     feederLdrCfg = new SparkMaxConfig();
//     //feederFlwCfg = new SparkMaxConfig();
//     // SoftLimitConfig leaderSoftLimit = new SoftLimitConfig();
//     // SoftLimitConfig followerSoftLimit = new SoftLimitConfig();

//     // intakeLdrPID = intakeMtrLdr.getClosedLoopController();
//     // intakeFlwPID = intakeMtrFlw.getClosedLoopController();

//     launcherEncLdr = launcherMtrLdr.getEncoder();
//     feederEncLdr = feederMtrLdr.getEncoder();
//     // intakeEncFlw = intakeMtrFlw.getEncoder();

//     launcherLdrCfg
//         .inverted(false)
//         .voltageCompensation(12.0)
//         .smartCurrentLimit(80)
//         .idleMode(IdleMode.kCoast);
//         // .closedLoop
//         //     .pid(kLdrP, kLdrI, kLdrD)
//         //     .outputRange(kLdrOutputMin, kLdrOutputMax)
//         //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//         //     .maxMotion
//         //         .maxAcceleration(kLdrMaxAccel)
//         //         .maxVelocity(kLdrMaxRPM);
//     launcherMtrLdr.configure(launcherLdrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     LauncherFlwCfg
//         .follow(launcherMtrLdr, false)
//         .voltageCompensation(12.0)
//         .smartCurrentLimit(80)
//         .idleMode(IdleMode.kCoast);
//         // .closedLoop
//         //     .pid(kFlwP, kFlwI, kFlwD)
//         //     .outputRange(kFlwOutputMin, kFlwOutputMax)
//         //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//         //     .maxMotion
//         //         .maxAcceleration(kFlwMaxAccel)
//         //         .maxVelocity(kFlwMaxRPM);
//     launcherMtrFlw.configure(LauncherFlwCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


//     feederLdrCfg
//         .inverted(false)
//         .voltageCompensation(12.0)
//         .smartCurrentLimit(80)
//         .idleMode(IdleMode.kCoast);
//     launcherMtrLdr.configure(launcherLdrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


//     // feederFlwCfg
//     //     .follow(feederMtrLdr, true)
//     //     .voltageCompensation(12.0)
//     //     .smartCurrentLimit(80)
//     //     .idleMode(IdleMode.kBrake);
//     // launcherMtrLdr.configure(launcherLdrCfg,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
//     if (Robot.isSimulation()) {
//       launcherMtrLdrSim = new SparkFlexSim(launcherMtrLdr, DCMotor.getNEO(1));
//       launcherMtrFlwSim = new SparkFlexSim(launcherMtrFlw, DCMotor.getNEO(1));
//       feederMtrLdrSim = new SparkMaxSim(feederMtrLdr, DCMotor.getNEO(1));
//       // feederMtrFlwSim = new SparkMaxSim(feederMtrFlw, DCMotor.getNEO(1));
//       launcherEncLdrSim = new SparkRelativeEncoderSim(launcherMtrLdr);
//       launcherEncFlwSim = new SparkRelativeEncoderSim(launcherMtrFlw);
//       feederEncLdrSim = new SparkRelativeEncoderSim(feederMtrLdr);
//       // feederEncFlwSim = new SparkRelativeEncoderSim(feederMtrFlw);
//       // leaderIntakeSim.setVelocity(0);
//       // followerIntakeSim.setVelocity(0);
//       // leaderEncoderSim.setVelocity(0);
//       // followerEncoderSim.setVelocity(0);
//     }

//   }

//   @Override
//   public void simulationPeriodic() {
//       // This method will be called once per scheduler run during simulation
//       if (Robot.isSimulation()) {
//           // leaderEncoderSim.setPosition(rotateMotorSim.getPosition());
//           launcherMtrLdrSim.iterate(launcherEncLdrSim.getVelocity(), launcherMtrLdrSim.getBusVoltage(),.005);
//           launcherMtrFlwSim.iterate(launcherEncFlwSim.getVelocity(), launcherMtrFlwSim.getBusVoltage(),.005);
//           feederMtrLdrSim.iterate(feederEncLdrSim.getVelocity(), feederMtrLdrSim.getBusVoltage(),.005);
//           //feederMtrFlwSim.iterate(feederEncFlwSim.getVelocity(), feederMtrFlwSim.getBusVoltage(),.005);
//       }
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     if (Robot.isSimulation()) {
//         SmartDashboard.putNumber("Launcher (%)", launcherEncLdrSim.getVelocity());
//         SmartDashboard.putNumber("Feeder (%)", feederEncLdrSim.getVelocity());
//         // SmartDashboard.putNumber("Intake Follower Speed (RPM)", intakeEncFlwSim.getPosition());
//     } else {
//         SmartDashboard.putNumber("Launcher Speed (%)", launcherEncLdr.getVelocity());
//         SmartDashboard.putNumber("Feeder Speed (%)", feederEncLdr.getVelocity());
//         // SmartDashboard.putNumber("Intake Follower Speed (RPM)", intakeEncFlw.getPosition());
//     }
//   }
  

//   // public void runIntake(double speed) {
//   //   intakeLdrPID.setReference(speed, SparkFlex.ControlType.kVelocity);
//   // }

//   // public Command IntakeCmd() {
//   //   return this.runEnd(
//   //       () -> {
//   //           // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
//   //           intakeMtrLdr.set(0.15);
//   //       },
//   //       () -> {
//   //           // runIntake(Constants.IntakeConstants.HOLD_SPEED);
//   //           intakeMtrLdr.set(0.1);
//   //       }
//   //     );
//   // }

//   public Command RunIntakeCmd() {
//     return this.runEnd(
//         () -> {
//             // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
//             launcherMtrLdr.set(-0.075);
//             feederMtrLdr.set(-0.2);
//         }, ()-> {
//           launcherMtrLdr.set(-0.025);
//             feederMtrLdr.set(-0.1);
//         }
//       );
//   }

//   public Command RunHoldCmd() {
//     return this.run(
//         () -> {
//             // runIntake(Constants.IntakeConstants.INTAKE_SPEED);
//             feederMtrLdr.set(0.25);
//         }
//       );
//   }

//   public Command RunOuttakeCmd() {
//     return this.runEnd(


//         () -> {
//             // runIntake(Constants.IntakeConstants.OUTTAKE_SPEED);
//             launcherMtrLdr.set(1.0);
//             if (launcherMtrLdr.getEncoder().getVelocity() >= 2250){
//                 feederMtrLdr.set(0.4);
//             }
//         }, () -> {
//           launcherMtrLdr.set(0.0);
//           feederMtrLdr.set(0.0);

//         }
//       );
//   }

//     // public Command StopCmd() {
//   //   return this.runOnce(
//   //       () -> {
//   //           runIntake(Constants.IntakeConstants.STOP_SPEED);
//   //       }
//   //     );
//   // }

// }






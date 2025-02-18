// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdvHdg;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.AlgaeIntakeSubsystem;
import frc.robot.subsystems.Secondary.AlgaeRotateSubsystem;
import frc.robot.subsystems.Secondary.CoralSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController engineerXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));


  private final AlgaeRotateSubsystem rotateSubsystem = new AlgaeRotateSubsystem();
  private final AlgaeIntakeSubsystem intakeSubsystem = new AlgaeIntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final CoralSubsystem coralSubsystem = new CoralSubsystem();   
  
  private final SendableChooser<Command> autoChooser;

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // rotation control is selectable between direct angle and angular velocity
  // left stick controls translation
  // in one mode the right stick controls the rotational velocity 
  // in the other mode the right stick controls the desired angle NOT angular rotation
  // also in this mode the POV buttons are used to quickly face a direction
  // and a button will yaw the robot towards a target.
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  Command AbsoluteDriveAdvHdg = new AbsoluteDriveAdvHdg(drivebase,
                                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                  OperatorConstants.LEFT_Y_DEADBAND) *
                                                                                                  DrivebaseConstants.Max_Speed_Multiplier,
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                  OperatorConstants.LEFT_X_DEADBAND) *
                                                                                                  DrivebaseConstants.Max_Speed_Multiplier,
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getRightX(),OperatorConstants.LEFT_X_DEADBAND),
                                                                    () -> -MathUtil.applyDeadband(driverXbox.getRightY(),OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> MathUtil.applyDeadband(driverXbox.getLeftTriggerAxis(),OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> MathUtil.applyDeadband(driverXbox.getRightTriggerAxis(),OperatorConstants.LEFT_Y_DEADBAND),
                                                                    () -> driverXbox.getHID().getPOV(),
                                                                    driverXbox.rightStick());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    drivebase.setDefaultCommand(AbsoluteDriveAdvHdg);


    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(AbsoluteDriveAdvHdg); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void initSlider(){
    // new SliderInitCmd(coralSubsystem).schedule();
    new CoralSubsystem().SliderInitCmd().schedule();
  }

  public void initElevator(){
    // new ElevatorInitCmd(elevatorSubsystem).schedule();
    new ElevatorSubsystem().ElevatorInitCmd().schedule();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}

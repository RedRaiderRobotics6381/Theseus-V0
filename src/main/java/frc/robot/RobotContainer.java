// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.CoralSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

import java.io.File;
import java.util.Optional;

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
  public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));


  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final CoralSubsystem coralSubsystem = new CoralSubsystem();   

  public boolean hdgModePressed = false; // Flag to track button state
  private double headingX = 0;
  private double headingY = 0;
  private Command currentDriveCmd = null;
  private final SendableChooser<Command> autoChooser;
  public double currentSnappedAngle = 0;
  public double snappedAngle = 0; 

  /**
   * Converts driver input into a robot-centric ChassisSpeeds that is controlled by the left and right triggers.
   */
  SwerveInputStream driveRobotCentricSideShift = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                      () -> 0,
                                                                      () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis())
                                                                      .withControllerRotationAxis(() -> 0)
                                                                      .deadband(OperatorConstants.DEADBAND)
                                                                      .scaleTranslation(1)
                                                                      .cubeTranslationControllerAxis(true)
                                                                      .cubeRotationControllerAxis(true)
                                                                      .headingWhile(true)
                                                                      .robotRelative(true)
                                                                      .allianceRelativeControl(false);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY(),
                                                                () -> -driverXbox.getLeftX())
                                                                .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(DrivebaseConstants.Max_Speed_Multiplier)
                                                                .cubeTranslationControllerAxis(true)
                                                                .cubeRotationControllerAxis(true)
                                                                .headingWhile(false)
                                                                .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngleSnapped = driveAngularVelocity.copy().withControllerHeadingAxis(
                                                                        () -> {
                                                                          headingX = driverXbox.getRightX();
                                                                          headingY = driverXbox.getRightY();
                                                                          if (driverXbox.getHID().getPOV() != -1) {
                                                                            return Math.sin(Math.toRadians(-driverXbox.getHID().getPOV()));
                                                                          } else {
                                                                            if (Math.sqrt(Math.pow(headingX, 2) + Math.pow(headingY, 2)) > 0.2) {
                                                                            return Math.sin(Math.toRadians((Math.round(((Math.toDegrees(Math.atan2(-driverXbox.getRightX(), -driverXbox.getRightY())) + 360) % 360) / 60.0) * 60.0)));
                                                                          }
                                                                          if(DriverStation.getAlliance().get() == Alliance.Blue){
                                                                            return Math.sin(Math.toRadians(Math.round((drivebase.getHeading().getDegrees() + 360) % 360)));
                                                                          }
                                                                          else{
                                                                            return Math.sin(Math.toRadians(Math.round((drivebase.getHeading().getDegrees() + 180) % 360)));
                                                                          }
                                                                          }}
                                                                        ,
                                                                        () -> {
                                                                          if (driverXbox.getHID().getPOV() != -1) {
                                                                            return Math.cos(Math.toRadians(driverXbox.getHID().getPOV()));
                                                                          } else {
                                                                            if (Math.sqrt(Math.pow(headingX, 2) + Math.pow(headingY, 2)) > 0.2) {
                                                                            return Math.cos(Math.toRadians((Math.round(((Math.toDegrees(Math.atan2(-driverXbox.getRightX(), -driverXbox.getRightY())) + 360) % 360) / 60.0) * 60.0)));
                                                                          }
                                                                          if(DriverStation.getAlliance().get() == Alliance.Blue){
                                                                            return Math.cos(Math.toRadians(Math.round((drivebase.getHeading().getDegrees() + 360) % 360)));
                                                                          }
                                                                          else {
                                                                            return Math.cos(Math.toRadians(Math.round((drivebase.getHeading().getDegrees() + 180) % 360)));
                                                                          }
                                                                        }})
                                                                      .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    autoChooser = AutoBuilder.buildAutoChooser();
    
    // NamedCommands.registerCommand("AlgaeIntakeRotate", rotateSubsystem.RotatePosCmd(Constants.AlgaeRotateConstants.ALGAE_INTAKE_POS));
    // NamedCommands.registerCommand("AlgaeBargeRotate", rotateSubsystem.RotatePosCmd(Constants.AlgaeRotateConstants.ALGAE_BARGE_POS));
    // NamedCommands.registerCommand("AlgaeIntake", intakeSubsystem.RunIntakeCmd());
    // NamedCommands.registerCommand("AlgaeHold", intakeSubsystem.RunHoldCmd());
    // NamedCommands.registerCommand("AlgaeOuttake", intakeSubsystem.RunOuttakeCmd());
    // NamedCommands.registerCommand("CoralIntakePos", rotateSubsystem.RotatePosCmd(Constants.CoralConstants.CORAL_START_ANGLE));
    // NamedCommands.registerCommand("CoralHighOuttake", rotateSubsystem.RotatePosCmd(Constants.CoralConstants.CORAL_HIGH_ANGLE));
    NamedCommands.registerCommand("ElevatorHigh", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants .REEF_HIGH_POSE));
    // NamedCommands.registerCommand("SliderLeft", coralSubsystem.setSliderPosition(Constants.CoralConstants.CORAL_SLIDER_LEFT_POSITION));
    // NamedCommands.registerCommand("SliderMiddle", coralSubsystem.setSliderPosition(Constants.CoralConstants.CORAL_SLIDER_MIDDLE_POSITION));
    // NamedCommands.registerCommand("SliderRight", coralSubsystem.setSliderPosition(Constants.CoralConstants.CORAL_SLIDER_RIGHT_POSITION));
    // NamedCommands.registerCommand("CoralIntake", coralSubsystem.IntakeCmd());
    // NamedCommands.registerCommand("CoralOuttake", coralSubsystem.OuttakeCmd());
    NamedCommands.registerCommand("ElevatorDown", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.START_POSE));
    NamedCommands.registerCommand("ElevatorBarge", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_HIGH_POSE));
    NamedCommands.registerCommand("ElevatorHighAlgae", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE));
    NamedCommands.registerCommand("ElevatorLowAlgae", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE));
    
    configureBindings();
    
    DriverStation.silenceJoystickConnectionWarning(true);
    
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
    Command driveFieldOrientedAngleSnapped  = drivebase.driveFieldOriented(driveDirectAngleSnapped);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedSideShift = drivebase.driveFieldOriented(driveRobotCentricSideShift);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    driverXbox.rightStick().onTrue(Commands.runOnce(() -> {
        hdgModePressed = !hdgModePressed;
        if (hdgModePressed){
          //heading = drivebase.getHeading().getDegrees();
          driveFieldOrientedAngleSnapped.schedule();
        }
        else{
          driveFieldOrientedAnglularVelocity.schedule();
        }
      }));

      driverXbox.leftTrigger(OperatorConstants.DEADBAND).or(driverXbox.rightTrigger(OperatorConstants.DEADBAND)).onTrue(Commands.runOnce(() -> {
        if(drivebase.getCurrentCommand() != driveRobotOrientedSideShift){
        currentDriveCmd = drivebase.getCurrentCommand();
        }
        driveRobotOrientedSideShift.schedule();
         }));
      
      driverXbox.leftTrigger(OperatorConstants.DEADBAND).negate().and(driverXbox.rightTrigger(OperatorConstants.DEADBAND).negate()).onTrue(Commands.runOnce(() -> {
          // count = 0;
          currentDriveCmd.schedule();
          // driveFieldOrientedAnglularVelocity.schedule();
      }));
      
      // driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()))));
      
      // Spencer Buttons Adjusts the maximum speed multiplier of the drivebase based on the state of the Xbox controller bumpers.
      driverXbox.rightBumper().onTrue(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(1);
                                                              driveAngularVelocity.scaleTranslation(1);}));
      driverXbox.rightBumper().negate().and(driverXbox.leftBumper().negate()).onTrue(Commands.runOnce(() -> {
                                                              driveAngularVelocity.scaleTranslation(0.75);
                                                              driveAngularVelocity.scaleTranslation(0.75);}));
      driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(0.5);
                                                             driveAngularVelocity.scaleTranslation(0.5);}));

      driverXbox.b().whileTrue(Commands.deferredProxy(() -> {
                                getSnappedAngleID();
                                return drivebase.driveToPose(
                                Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                                new Transform2d(0.6604,   -.164338,
                                Rotation2d.fromDegrees(180))));
                              }));
      driverXbox.x().whileTrue(Commands.deferredProxy(() -> {
                                getSnappedAngleID();
                                return drivebase.driveToPose(
                                Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                                new Transform2d(0.6604,   .164338,
                                Rotation2d.fromDegrees(180))));
                              }));
      driverXbox.y().whileTrue(Commands.deferredProxy(() -> {
                                getSnappedAngleID();
                                return drivebase.driveToPose(
                                Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                                new Transform2d(0.6604,   0.0,
                                Rotation2d.fromDegrees(180))));
                              }));
      driverXbox.a().whileTrue(Commands.deferredProxy(() -> {
                                // getSnappedAngleID();
                                return drivebase.driveToPose(
                                Vision.getAprilTagPose(AprilTagConstants.HumanPlayerLeft,
                                new Transform2d(0.6604,   0.0,
                                Rotation2d.fromDegrees(180.0))));
                              }));

      // driverXbox.pov(0).onTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
      //                       new Pose2d(1.0,  0.0,
      //                       Rotation2d.fromDegrees(0.0)))));

      // driverXbox.pov(180).onTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
      //                       new Pose2d(-1.0,   0.0,
      //                       Rotation2d.fromDegrees(0.0)))));

      // driverXbox.pov(90).onTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
      //                       new Pose2d(0.0,   0.0,
      //                       Rotation2d.fromDegrees(0.0)))));

      // driverXbox.pov(270).onTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
      //                       new Pose2d(0.0,   0.0,
      //                       Rotation2d.fromDegrees(180.0)))));

      // engineerXbox.b().whileTrue(rotateSubsystem.RotateCmd());

      // engineerXbox.a().onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE));
      // engineerXbox.a().onTrue(Commands.sequence(
      //   Commands.parallel(
      //       elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE), 
      //       coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_START_ANGLE))
      //   )
      //   .andThen(
      //       intakeSubsystem.RunIntakeCmd()));

      engineerXbox.leftBumper().onTrue(coralSubsystem.IntakeCmd());
      engineerXbox.rightBumper().onTrue(coralSubsystem.OuttakeCmd());

      // engineerXbox.leftStick().whileTrue(new PositionIdentifierCmd(   elevatorSubsystem,
      //                                                                 coralSubsystem, 
      //                                                                 () -> engineerXbox.getLeftX(),
      //                                                                 () -> engineerXbox.getLeftY()));

      // engineerXbox.b().onTrue(coralSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_HIGH_ANGLE));

      // engineerXbox.x().onTrue(coralSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_START_ANGLE)); // change to whiletrue

      // engineerXbox.y().onTrue(coralSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_LOW_ANGLE));

      // engineerXbox.x().onTrue(coralSubsystem.IntakeCmd());
      // engineerXbox.b().onTrue(coralSubsystem.OuttakeCmd());
      // engineerXbox.a().onTrue(Commands.sequence(
      //   Commands.parallel(
      //     coralSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_START_ANGLE),
      //     elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.START_POSE)
      // )));

      // engineerXbox.a().onTrue(elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_MIDDLE_POSE));

      // engineerXbox.y().onTrue(elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.START_POSE));

      // engineerXbox.x().onTrue(rotateSubsystem.RotatePosCmd(Constants.AlgaeRotateConstants.ALGAE_START_POS));

      // engineerXbox.b().onTrue(elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.START_POSE));


      // engineerXbox.a().onTrue(Commands.sequence(
      //   Commands.parallel(
      //     elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE), 
      //     coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_START_ANGLE)))
      //   .andThen(
      //     intakeSubsystem.RunIntakeCmd()));

      // engineerXbox.povLeft().onTrue(Commands.sequence(
      //   Commands.parallel(
      //       elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.ALGAE_PICKUP_LOW_POSE), 
      //       rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_INTAKE_POS),
      //       intakeSubsystem.RunIntakeCmd()
      //   )
      //   ));

      // engineerXbox.povRight().onTrue(Commands.sequence(
      //   Commands.parallel(
      //       elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.ALGAE_PICKUP_HIGH_POSE), 
      //       rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_INTAKE_POS),
      //       intakeSubsystem.RunIntakeCmd()
      //   )
      //   ));

      // engineerXbox.povUp().onTrue(Commands.sequence(
      //   Commands.parallel(
      //       elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_HIGH_POSE), 
      //       rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_BARGE_POS)
      //   )
      //   ));

      // engineerXbox.povDown().onTrue(Commands.sequence(
      //   Commands.parallel(
      //       elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.ALGAE_PROCESSOR_POSE), 
      //       rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_INTAKE_POS)
      //   )
      //   ));
      

        //NOTHING IS WORKING, I AM SETTING TEMPORARY BUTTONS FOR DULUTH. 

        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.a())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE)); 
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.x())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_LOW_POSE));
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.y())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_MIDDLE_POSE));
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.b())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_HIGH_POSE)); 

        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.b())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_HIGH_ANGLE));
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.x())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_LOW_ANGLE)); 
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.y())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_MID_ANGLE)); 
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.a())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_START_ANGLE)); 

        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.b())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_INTAKE_POS));
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.x())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_INTAKE_POS));
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.y())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_BARGE_POS));
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.a())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_PROCESSOR_POS));

        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.leftBumper())).onTrue(coralSubsystem.IntakeCmd());
        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.rightBumper())).onTrue(coralSubsystem.OuttakeCmd()); 

        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.leftBumper())).whileTrue(intakeSubsystem.RunIntakeCmd());
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.rightBumper())).whileTrue(intakeSubsystem.RunOuttakeCmd()); 
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

  // public void initSlider(){
  //   // new SliderInitCmd(coralSubsystem).schedule();
  //   coralSubsystem.SliderInitCmd().schedule();
  // }

  // public void initElevator(){
  //   // new ElevatorInitCmd(elevatorSubsystem).schedule();
  //   elevatorSubsystem.ElevatorInitCmd().schedule();
  // }

  // public void initCoralRotate() {
  //   coralSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_START_ANGLE).schedule();
  // }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  void getSnappedAngleID(){
    Rotation2d currentHeading = drivebase.getHeading();   
    double angle = Math.toDegrees(Math.atan2(currentHeading.getCos(), currentHeading.getSin()))-270;
    // Normalize to the range [0, 360)
    angle = (angle + 360) % 360;

    // Snap to the nearest 60-degree increment
    snappedAngle = Math.round(angle / 60.0) * 60.0;
    if (snappedAngle != currentSnappedAngle){
      currentSnappedAngle = snappedAngle;
      
      // if (headingX != 0 || headingY != 0) {
      //   headingX = Math.sin(Math.toRadians(snappedAngle));
      //   headingY = Math.cos(Math.toRadians(snappedAngle));

      Optional<Alliance> allianceColor = DriverStation.getAlliance();
      if (allianceColor.isPresent()) {
        if (allianceColor.get() == Alliance.Red) {
          if(snappedAngle == 60.0){AprilTagConstants.ReefTagID = 6;};
          if(snappedAngle == 0.0){AprilTagConstants.ReefTagID = 7;};
          if(snappedAngle == -60.0){AprilTagConstants.ReefTagID = 8;};
          if(snappedAngle == 240.0){AprilTagConstants.ReefTagID = 9;};
          if(snappedAngle == 120.0){AprilTagConstants.ReefTagID = 11;};
          if(snappedAngle == 180.0){AprilTagConstants.ReefTagID = 10;};

        }
        else if (allianceColor.get() == Alliance.Blue) {
          if(snappedAngle == 60.0){AprilTagConstants.ReefTagID = 22;};
          if(snappedAngle == 180.0){AprilTagConstants.ReefTagID = 18;};
          if(snappedAngle == -60.0){AprilTagConstants.ReefTagID = 20;};
          if(snappedAngle == 240.0){AprilTagConstants.ReefTagID = 19;};
          if(snappedAngle == 120.0){AprilTagConstants.ReefTagID = 17;};
          if(snappedAngle == 0.0){AprilTagConstants.ReefTagID = 21;};
        }
      }

    }
    System.out.println("Snapped Angle: " + snappedAngle);
    System.out.println("Reef Tag: " + AprilTagConstants.ReefTagID);
  }
}

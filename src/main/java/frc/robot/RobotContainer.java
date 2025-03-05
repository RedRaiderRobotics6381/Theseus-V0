// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;
// import frc.robot.commands.Secondary.PositionIdentifierCmd;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdvHdg;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
// import frc.robot.subsystems.Secondary.AlgaeIntakeSubsystem;
// import frc.robot.subsystems.Secondary.AlgaeRotateSubsystem;
// import frc.robot.subsystems.Secondary.CoralSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;

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


  // public final AlgaeRotateSubsystem rotateSubsystem = new AlgaeRotateSubsystem();
  // public final AlgaeIntakeSubsystem intakeSubsystem = new AlgaeIntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  // public final CoralSubsystem coralSubsystem = new CoralSubsystem();   

  public boolean hdgModePressed = false; // Flag to track button state
  private double headingX = 0;
  private double headingY = 0;
  private double heading = 0;
  private Command currentDriveCmd = null;
  
  private final SendableChooser<Command> autoChooser;
  private int count = 0;

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
  // Command AbsoluteDriveAdvHdg = new AbsoluteDriveAdvHdg(drivebase,
  //                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
  //                                                                                                 OperatorConstants.LEFT_Y_DEADBAND) *
  //                                                                                                 DrivebaseConstants.Max_Speed_Multiplier,
  //                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
  //                                                                                                 OperatorConstants.LEFT_X_DEADBAND) *
  //                                                                                                 DrivebaseConstants.Max_Speed_Multiplier,
  //                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),OperatorConstants.LEFT_X_DEADBAND),
  //                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightY(),OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                   () -> MathUtil.applyDeadband(driverXbox.getLeftTriggerAxis(),OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                   () -> MathUtil.applyDeadband(driverXbox.getRightTriggerAxis(),OperatorConstants.LEFT_Y_DEADBAND),
  //                                                                   () -> driverXbox.getHID().getPOV(),
  //                                                                   driverXbox.rightStick());

  /**
   * Converts driver input into a robot-centric ChassisSpeeds that is controlled by the left and right triggers.
   */
  SwerveInputStream driveRobotCentricSideShift = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                () -> 0,
                                () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis())
                              .withControllerRotationAxis(() -> 0)
                              .deadband(OperatorConstants.DEADBAND)
                              .scaleTranslation(0.5)
                              .cubeTranslationControllerAxis(true)
                              .cubeRotationControllerAxis(true)
                              .headingWhile(false)
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
                                                            // .scaleRotation(DrivebaseConstants.Max_Speed_Multiplier)
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
                                                                          // angle = ((Math.toDegrees(Math.atan2(headingX, headingY)) + 360) % 360);
                                                                          // snappedAngle = (Math.round(((Math.toDegrees(Math.atan2(headingX, headingY)) + 360) % 360) / 60.0) * 60.0);
                                                                          if (driverXbox.getHID().getPOV() != -1) {
                                                                            return Math.sin(Math.toRadians(-driverXbox.getHID().getPOV()));
                                                                          } else {
                                                                            if (Math.sqrt(Math.pow(headingX, 2) + Math.pow(headingY, 2)) > 0.2) {
                                                                            return Math.sin(Math.toRadians((Math.round(((Math.toDegrees(Math.atan2(-driverXbox.getRightX(), -driverXbox.getRightY())) + 360) % 360) / 60.0) * 60.0)));
                                                                          }
                                                                          if(DriverStation.getAlliance().get() == Alliance.Blue){
                                                                            return Math.sin(getSnappedAngle(drivebase.getHeading().getDegrees()));
                                                                          }
                                                                          else{
                                                                            return Math.sin(getSnappedIdleAngle(drivebase.getHeading().getDegrees()));
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
                                                                            return Math.cos(getSnappedAngle(drivebase.getHeading().getDegrees()));
                                                                          }
                                                                          else {
                                                                            return Math.cos(getSnappedIdleAngle(drivebase.getHeading().getDegrees()));
                                                                          }
                                                                        }})
                                                                      .headingWhile(true);

  public double getSnappedAngle(double heading){
    return Math.toRadians(Math.round((heading + 360) % 360));
  }
  public double getSnappedIdleAngle(double heading){
    return Math.toRadians(Math.round((heading + 180) % 360));
  }

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


    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
    }
    if (DriverStation.isTest())
    { // Overrides drive command above!

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.none());
      // driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
            driverXbox.back().whileTrue(drivebase.centerModulesCommand());

      driverXbox.rightStick().onTrue(Commands.runOnce(() -> {
        hdgModePressed = !hdgModePressed;
        if (hdgModePressed){
          heading = drivebase.getHeading().getDegrees();
          driveFieldOrientedAngleSnapped.schedule();
        }
        else{
          driveFieldOrientedAnglularVelocity.schedule();
        }
      }));

      driverXbox.leftTrigger(OperatorConstants.DEADBAND).or(driverXbox.rightTrigger(OperatorConstants.DEADBAND)).onTrue(Commands.runOnce(() -> {
        currentDriveCmd = drivebase.getCurrentCommand();
        driveRobotOrientedSideShift.schedule();
         }));
      // driverXbox.leftTrigger(OperatorConstants.DEADBAND).or(driverXbox.rightTrigger(OperatorConstants.DEADBAND)).whileTrue(Commands.run(() -> {
      //   if(count == 0){
      //   currentDriveCmd = drivebase.getCurrentCommand();
      //   count = 1;
      //   }
      //   if(!coralSubsystem.close){
      //   driveRobotOrientedSideShift.schedule();
      //   }else{
      //     driveFieldOrientedAnglularVelocity.schedule();
      //   }
      // }));
      
      driverXbox.leftTrigger(OperatorConstants.DEADBAND).or(driverXbox.rightTrigger(OperatorConstants.DEADBAND)).onFalse(Commands.runOnce(() -> {
          // count = 0;
          driveFieldOrientedAnglularVelocity.schedule();
        
      }));
      
       driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()))));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );

      driverXbox.b().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                          new Transform2d(0.6604,   -.164338,
                          Rotation2d.fromDegrees(180))))));

      driverXbox.x().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                          new Transform2d(0.6604,   .164338,
                          Rotation2d.fromDegrees(180))))));

      driverXbox.y().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                          new Transform2d(0.5,   0.0,
                          Rotation2d.fromDegrees(0.0))))));

      driverXbox.a().whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
                          Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                          new Transform2d(1.0,   0.0,
                          Rotation2d.fromDegrees(180.0))))));

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

      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());

      // engineerXbox.b().whileTrue(rotateSubsystem.RotateCmd());

      //engineerXbox.a().onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE));
      // engineerXbox.a().onTrue(Commands.sequence(
      //   Commands.parallel(
      //       elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE), 
      //       coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_START_ANGLE))
      //   )
      //   .andThen(
      //       intakeSubsystem.RunIntakeCmd()));

      // engineerXbox.leftBumper().whileTrue(intakeSubsystem.RunIntakeCmd());
      // engineerXbox.rightBumper().whileTrue(intakeSubsystem.RunOuttakeCmd());

      // engineerXbox.leftStick().whileTrue(new PositionIdentifierCmd(   elevatorSubsystem,
      //                                                                 coralSubsystem, 
      //                                                                 () -> engineerXbox.getLeftX(),
      //                                                                 () -> engineerXbox.getLeftY()));

      //engineerXbox.x().onTrue(Commands.run(() -> coralSubsystem.setRotateAngle(Constants.CoralConstants.CORAL_HIGH_ANGLE), coralSubsystem));

      //engineerXbox.a().onTrue(Commands.run(() -> coralSubsystem.setRotateAngle(Constants.CoralConstants.CORAL_START_ANGLE), coralSubsystem)); // change to whiletrue

      //engineerXbox.b().onTrue(Commands.run(() -> coralSubsystem.setRotateAngle(Constants.CoralConstants.CORAL_LOW_ANGLE), coralSubsystem));

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

        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.a())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(ElevatorConstants.START_POSE))); 
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.x())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(ElevatorConstants.REEF_LOW_POSE)));
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.y())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(ElevatorConstants.REEF_MIDDLE_POSE)));
        engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.b())).onTrue(Commands.run(() -> elevatorSubsystem.setElevatorHeight(ElevatorConstants.REEF_HIGH_POSE))); 

        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.b())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_HIGH_ANGLE));
        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.x())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_LOW_ANGLE)); 
        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.y())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_LOW_ANGLE)); 
        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.a())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_START_ANGLE)); 

        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.b())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_INTAKE_POS));
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.x())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_INTAKE_POS));
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.y())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_BARGE_POS));
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.a())).onTrue(rotateSubsystem.RotatePosCmd(AlgaeRotateConstants.ALGAE_PROCESSOR_POS));

        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.leftBumper())).onTrue(coralSubsystem.IntakeCmd());
        // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.rightBumper())).onTrue(coralSubsystem.OuttakeCmd()); 

        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.leftBumper())).whileTrue(intakeSubsystem.RunIntakeCmd());
        // engineerXbox.leftStick().negate().and(engineerXbox.rightStick().and(engineerXbox.rightBumper())).whileTrue(intakeSubsystem.RunOuttakeCmd()); 
    }

  }

  public void spencerButtons(){

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("HighSpd");
      DrivebaseConstants.Max_Speed_Multiplier = 1;
    }

    if (driverXbox.getHID().getRightBumper() == true && driverXbox.getHID().getLeftBumper() == false ||
        driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == true){
      //System.out.println("MedSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .875;
    }

    if (driverXbox.getHID().getRightBumper() == false && driverXbox.getHID().getLeftBumper() == false){
      //System.out.println("LowSpd");
      DrivebaseConstants.Max_Speed_Multiplier = .75;
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
}

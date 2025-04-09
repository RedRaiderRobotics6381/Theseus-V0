// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.commands.Secondary.ClimbCmd;
import frc.robot.commands.Secondary.PositionIdentifierCmd;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.IndexerSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;
import frc.robot.subsystems.Secondary.SliderSubsystem;
import frc.robot.subsystems.Secondary.ClimberSubsystem;
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
  public final RotateSubsystem rotateSubsystem = new RotateSubsystem();
  public final SliderSubsystem sliderSubsystem = new SliderSubsystem();
  public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();   

  private final SendableChooser<Command> autoChooser;
  public double currentSnappedAngle = 0;
  public double snappedAngle = 0;
  public boolean snappedAngleStart = true; 

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY(),
                                                                () -> -driverXbox.getLeftX() - driverXbox.getRightTriggerAxis() + driverXbox.getLeftTriggerAxis())
                                                                .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(DrivebaseConstants.Max_Speed_Multiplier)
                                                                .cubeTranslationControllerAxis(true)
                                                                .cubeRotationControllerAxis(true)
                                                                .headingWhile(false)
                                                                .allianceRelativeControl(true)
                                                                .robotRelative(false);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings

    configureBindings();
  
    NamedCommands.registerCommand("SliderLeft", sliderSubsystem.setSliderPositionCmd(Constants.CoralConstants.CORAL_SLIDER_LEFT_POSITION));
    NamedCommands.registerCommand("SliderMiddle", sliderSubsystem.setSliderPositionCmd(Constants.CoralConstants.CORAL_SLIDER_MIDDLE_POSITION));
    NamedCommands.registerCommand("SliderRight", sliderSubsystem.setSliderPositionCmd(Constants.CoralConstants.CORAL_SLIDER_RIGHT_POSITION));
    NamedCommands.registerCommand("CoralIntake", indexerSubsystem.IntakeCmd());
    NamedCommands.registerCommand("CoralOuttake", indexerSubsystem.OuttakeCmd());
    NamedCommands.registerCommand("CoralRotateL2", rotateSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_L2_L3_ANGLE));
    NamedCommands.registerCommand("CoralRotateL3", rotateSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_L2_L3_ANGLE));
    NamedCommands.registerCommand("CoralRotateL4", rotateSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_L4_ANGLE));
    NamedCommands.registerCommand("CoralRotateStart", rotateSubsystem.setRotateAngleCmd(Constants.CoralConstants.CORAL_START_ANGLE));
    NamedCommands.registerCommand("AlgaeRotateIntake", rotateSubsystem.setRotateAngleCmd(Constants.CoralConstants.ALGAE_INTAKE_ANGLE));
    NamedCommands.registerCommand("AlgaeRotateScore", rotateSubsystem.setRotateAngleCmd(Constants.CoralConstants.ALGAE_SCORE_ANGLE));
    NamedCommands.registerCommand("ElevatorStart", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.START_POSE));
    NamedCommands.registerCommand("ElevatorL2", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_L2_POSE));
    NamedCommands.registerCommand("ElevatorL3", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_L3_POSE));
    NamedCommands.registerCommand("ElevatorL4", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_L4_POSE));
    NamedCommands.registerCommand("ElevatorInit", elevatorSubsystem.ElevatorInitCmd());
    NamedCommands.registerCommand("SliderInit", sliderSubsystem.SliderInitCmd());
    NamedCommands.registerCommand("AlgaeIntake", indexerSubsystem.algaeIntakeCmd());
    NamedCommands.registerCommand("AlgaeOuttake", indexerSubsystem.algaeOuttakeCmd());
    NamedCommands.registerCommand("AlgaeIntakeLowElevator", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE));
    NamedCommands.registerCommand("AlgaeIntakeHighElevator", elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE));

    
    
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = AutoBuilder.buildAutoChooser();
    
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
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    // driverXbox.rightStick().whileTrue(Commands.runOnce(() -> {driveAngularVelocity.allianceRelativeControl(false);
    //                                                            driveAngularVelocity.robotRelative(true);}));
    // driverXbox.rightStick().onFalse(Commands.runOnce(() -> {driveAngularVelocity.robotRelative(false);
    //                                                         driveAngularVelocity.allianceRelativeControl(true);}));

    // driverXbox.rightStick().whileTrue(Commands.runOnce(() -> {driveAngularVelocity.cubeTranslationControllerAxis(false);})); //TODO: Check
    // driverXbox.rightStick().onFalse(Commands.runOnce(() -> {driveAngularVelocity.cubeTranslationControllerAxis(true);})); //TODO: Check
    
    // Spencer Buttons Adjusts the maximum speed multiplier of the drivebase based on the state of the Xbox controller bumpers.
    driverXbox.rightBumper().onTrue(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(1);}));
    driverXbox.rightBumper().negate().and(driverXbox.leftBumper().negate()).onTrue(Commands.runOnce(() -> {
                                                          driveAngularVelocity.scaleTranslation(0.75);}));
    driverXbox.leftBumper().onTrue(Commands.runOnce(() -> {driveAngularVelocity.scaleTranslation(0.5);}));


    driverXbox.a().whileTrue(Commands.deferredProxy(() -> {
                              // getSnappedAngleID();
                              return drivebase.driveToPoseWithConstraints(
                              Vision.getAprilTagPose(AprilTagConstants.HumanPlayerLeft,
                              new Transform2d(0.55,   0.0,
                              Rotation2d.fromDegrees(180.0))),
                              new PathConstraints(AutonConstants.LINEAR_VELOCITY,
                                                  AutonConstants.LINEAR_ACELERATION,
                                                  Math.toRadians(AutonConstants.ANGULAR_VELOCITY),
                                                  Math.toRadians(AutonConstants.ANGULAR_ACCELERATION)));
                            }));
    driverXbox.b().whileTrue(Commands.deferredProxy(() -> {
                              getSnappedAngleID();
                              // right = true;
                              return drivebase.driveToPoseWithConstraints(
                              Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                              new Transform2d(0.5,   0.16430625,
                              Rotation2d.fromDegrees(180))),
                              new PathConstraints(AutonConstants.LINEAR_VELOCITY,
                                                  AutonConstants.LINEAR_ACELERATION,
                                                  Math.toRadians(AutonConstants.ANGULAR_VELOCITY),
                                                  Math.toRadians(AutonConstants.ANGULAR_ACCELERATION)));
                            }));

    driverXbox.x().whileTrue(Commands.deferredProxy(() -> {
                              getSnappedAngleID();
                              // right = false;
                              return drivebase.driveToPoseWithConstraints(
                              Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                              new Transform2d(0.5,   -0.16430625,
                              Rotation2d.fromDegrees(180))),
                              new PathConstraints(AutonConstants.LINEAR_VELOCITY,
                                                  AutonConstants.LINEAR_ACELERATION,
                                                  Math.toRadians(AutonConstants.ANGULAR_VELOCITY),
                                                  Math.toRadians(AutonConstants.ANGULAR_ACCELERATION)));
                            }));
    driverXbox.y().whileTrue(Commands.deferredProxy(() -> {
                              getSnappedAngleID();
                              return drivebase.driveToPoseWithConstraints(
                              Vision.getAprilTagPose(AprilTagConstants.ReefTagID,
                              new Transform2d(0.5,   0.0,
                              Rotation2d.fromDegrees(180))),
                              new PathConstraints(AutonConstants.LINEAR_VELOCITY,
                                                  AutonConstants.LINEAR_ACELERATION,
                                                  Math.toRadians(AutonConstants.ANGULAR_VELOCITY),
                                                  Math.toRadians(AutonConstants.ANGULAR_ACCELERATION)));
                            }));
    
    engineerXbox.x().whileTrue(Commands.run(() -> {
      sliderSubsystem.setSliderPosition(getSliderOffset(false));
      // sliderSubsystem.setSliderPosition(getSliderOffset(6.46875, 8));
    }));

    engineerXbox.b().whileTrue(Commands.run(() -> {
      sliderSubsystem.setSliderPosition(getSliderOffset(true));
      // sliderSubsystem.setSliderPosition(getSliderOffset(-6.46875, 2));
    }));
      
    engineerXbox.leftTrigger(OperatorConstants.DEADBAND).whileTrue(
      Commands.run(() -> sliderSubsystem.sliderManual(engineerXbox.getLeftTriggerAxis()*.4)));
    
    engineerXbox.rightTrigger(OperatorConstants.DEADBAND).whileTrue(
      Commands.run(() -> sliderSubsystem.sliderManual(-engineerXbox.getRightTriggerAxis()*.4)));
    
    engineerXbox.leftTrigger(OperatorConstants.DEADBAND).negate().and(
      engineerXbox.rightTrigger(OperatorConstants.DEADBAND).negate()).onTrue(
      Commands.runOnce(() -> sliderSubsystem.sliderManual(0)));

    
    engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate()).and(engineerXbox.leftBumper()).onTrue(indexerSubsystem.IntakeCmd());
    engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate()).and(engineerXbox.rightBumper()).onTrue(indexerSubsystem.OuttakeCmd());

    engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.leftBumper())).whileTrue(Commands.run(() -> {
                                                                                                              if (!indexerSubsystem.coralSensor.get()) {
                                                                                                                  indexerSubsystem.algaeOuttakeCmd();}}));
    engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.rightBumper())).whileTrue(Commands.run(() -> {
                                                                                                              if (!indexerSubsystem.coralSensor.get()) {
                                                                                                                  indexerSubsystem.algaeIntakeCmd();}}));
    engineerXbox.leftStick().whileTrue(new PositionIdentifierCmd(   elevatorSubsystem,
                                                                    rotateSubsystem, 
                                                                    () -> engineerXbox.getLeftX(),
                                                                    () -> engineerXbox.getLeftY()));



                                                                    // engineerXbox.leftStick().onTrue(Commands.runOnce(() -> getSnappedAngleID()));


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

    engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate()).and(engineerXbox.povLeft()).onTrue(Commands.sequence(
      Commands.parallel(
          elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.ALGAE_PICKUP_LOW_POSE), 
          rotateSubsystem.setRotateAngleCmd(CoralConstants.ALGAE_INTAKE_ANGLE)
      )
      ));

      engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate()).and(engineerXbox.povRight()).onTrue(Commands.sequence(
      Commands.sequence(
          elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.ALGAE_PICKUP_HIGH_POSE), 
          rotateSubsystem.setRotateAngleCmd(CoralConstants.ALGAE_INTAKE_ANGLE)
      )
      ));

      engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate()).and(engineerXbox.povUp()).onTrue(Commands.sequence(
      Commands.parallel(
        elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.ALGAE_BARGE_POSE), 
        rotateSubsystem.setRotateAngleCmd(CoralConstants.ALGAE_SCORE_ANGLE)
      )
      ));
    

      //NOTHING IS WORKING, I AM SETTING TEMPORARY BUTTONS FOR DULUTH. 

      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.a())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE)); 
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.x())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_LOW_POSE));
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.y())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_MIDDLE_POSE));
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.b())).onTrue(elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_HIGH_POSE)); 

      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.b())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_HIGH_ANGLE));
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.x())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_LOW_ANGLE)); 
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.y())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_MID_ANGLE)); 
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.a())).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_START_ANGLE));

      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.pov(90))).onTrue(coralSubsystem.setSliderPositionCmd(-0.5));
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().negate().and(engineerXbox.pov(270))).onTrue(coralSubsystem.setSliderPositionCmd(-12.5));

      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.pov(0))).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.ALGAE_SCORE_ANGLE));
      // engineerXbox.rightStick().negate().and(engineerXbox.leftStick().and(engineerXbox.pov(180))).onTrue(coralSubsystem.setRotateAngleCmd(CoralConstants.ALGAE_INTAKE_ANGLE));
      engineerXbox.rightStick().and(engineerXbox.leftStick().negate()).and(engineerXbox.povRight()).onTrue(climberSubsystem.climbAndGetPaid(310.0));
      engineerXbox.rightStick().and(engineerXbox.leftStick().negate()).and(engineerXbox.povLeft()).onTrue(new ClimbCmd(climberSubsystem, rotateSubsystem, sliderSubsystem));

      engineerXbox.rightStick().and(engineerXbox.leftStick().negate()).and(engineerXbox.leftBumper()).whileTrue(indexerSubsystem.Retract());
      engineerXbox.rightStick().and(engineerXbox.leftStick().negate()).and(engineerXbox.rightBumper()).whileTrue(indexerSubsystem.AccurateOuttake());
      

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
    if (snappedAngle != currentSnappedAngle || snappedAngleStart){
      snappedAngleStart = false;
      
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
      currentSnappedAngle = snappedAngle;
    }
    SmartDashboard.putNumber("Snapped Angle: ", snappedAngle);
    SmartDashboard.putNumber("Reef Tag ID: ", AprilTagConstants.ReefTagID);
  }
  // double getSliderOffset(double reefOffset, double controlOffset){
    double getSliderOffset(boolean right){
    getSnappedAngleID();
    Pose2d tagPose;
    Pose2d robotPose = drivebase.getPose();
    double tagX;
    double tagY;
    double robotX = robotPose.getTranslation().getX();
    double robotY = robotPose.getTranslation().getY();
    int alternator = 1;
    if(right){
    tagPose = Vision.getAprilTagPose(AprilTagConstants.ReefTagID, new Transform2d(0.43, 0.16430, Rotation2d.fromDegrees(180)));


    } else {
      tagPose = Vision.getAprilTagPose(AprilTagConstants.ReefTagID, new Transform2d(0.43, -0.16430, Rotation2d.fromDegrees(180)));
    }
    tagX = tagPose.getTranslation().getX();
    tagY = tagPose.getTranslation().getY();
  
    double deltaX = robotX - tagX;
    double deltaY = robotY - tagY;
    
    double yOffset = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    if (snappedAngle == 180 || snappedAngle == 120 || snappedAngle == 240){
      if (tagY < robotY){
          alternator = -1;
      }
    } else {
      if(tagY > robotY){
        alternator = -1;
      }
    }
     

    yOffset = alternator * yOffset;
    // 6.46875 is the distance from the center of the robot to the center of the coral slider 6 is the center of the slider
    yOffset = 6 + Units.metersToInches(yOffset);
    // Clamp yOffset between minOffset and maxOffset
    double minOffset = 0.0; // Set your minimum offset value here
    double maxOffset = 12.0; // Set your maximum offset value here
    yOffset = Math.max(minOffset, Math.min(yOffset, maxOffset));

    // SmartDashboard.putNumber("X Offset To Tag", Units.metersToInches(xOffset));
    SmartDashboard.putNumber("Slider Offset", -yOffset);
    SmartDashboard.putNumber("deltaX", deltaX);
    SmartDashboard.putNumber("deltaY", deltaY);
    SmartDashboard.putNumber("Tag X", tagX);
    SmartDashboard.putNumber("Tag Y", tagY);
    return -yOffset;
    // getSnappedAngleID();
    // Pose2d robotPose = drivebase.getPose();
    // Pose2d tagPose = Vision.getAprilTagPose(AprilTagConstants.ReefTagID, new Transform2d(0.435, 0.0, Rotation2d.fromDegrees(180)));

    // double deltaX = robotPose.getTranslation().getX() - tagPose.getTranslation().getX();
    // // if (reefOffset < 0.0){
    // //   reefOffset = reefOffset * -1;
    // // }
    // double deltaY = robotPose.getTranslation().getY() - tagPose.getTranslation().getY() + Units.inchesToMeters(reefOffset);

    // // double robotHeading = robotPose.getRotation().getRadians();
    // double tagHeading = tagPose.getRotation().getRadians();

    // // double xOffset = deltaX * Math.cos(tagHeading) + deltaY * Math.sin(tagHeading);
    // double yOffset = -deltaX * Math.sin(tagHeading) + deltaY * Math.cos(tagHeading);
    // // 6.46875 is the distance from the center of the robot to the center of the coral slider 6 is the center of the slider
    // yOffset = controlOffset - Units.metersToInches(yOffset);
    // // Clamp yOffset between minOffset and maxOffset
    // double minOffset = 0.0; // Set your minimum offset value here
    // double maxOffset = 12.0; // Set your maximum offset value here
    // yOffset = Math.max(minOffset, Math.min(yOffset, maxOffset));

    // // SmartDashboard.putNumber("X Offset To Tag", Units.metersToInches(xOffset));
    // SmartDashboard.putNumber("Slider Offset", -yOffset);
    // SmartDashboard.putNumber("deltaX", deltaX);
    // SmartDashboard.putNumber("deltaY", deltaY);
    // return -yOffset;
    
  }
}

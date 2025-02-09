// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;


public class AbsoluteDriveAdvHdg extends Command
{

  private final SwerveSubsystem swerve;
  private final DoubleSupplier  vX, vY;
  private final DoubleSupplier  oX, oY;
  private final DoubleSupplier leftY, rightY;
  private final DoubleSupplier lookPOV;
  private final BooleanSupplier hdgMode;

  boolean hdgModePressed = false; // Flag to track button state
  boolean angHdgMode = false; // Flag to track angle mode
  boolean angVelMode = true; // Flag to track velocity mode (default)
  boolean hdgPOV = false; // Flag to track POV mode
  boolean resetHeading = false; // Flag to track heading reset 


  /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. There is a mode button that will allow selection of the
   * heading to be controlled by angular heading or angular velocity. The look booleans are shortcuts to get the robot to
   * face a certian direction. With an extra look boolean that faces the robot towards the intended vision target.
   * This is a mashup of driveFieldOrientedAnglularVelocity and AbsoluteDriveAdv. 
   *
   * @param swerve        The swerve drivebase subsystem.
   * @param vX            DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY            DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1
   *                      with deadband already accounted for.  Positive Y is towards the left wall when looking through
   *                      the driver station glass.
   * @param oX            DoubleSupplier that supplies the x-orientation joystick input.  Should be in the range -1 to 1
   * @param oY            DoubleSupplier that supplies the y-orientation joystick input.  Should be in the range -1 to 1
   * @param lookAway      Face the robot towards the opposing alliance's wall in the same direction the driver is
   *                      facing
   * @param lookTowards   Face the robot towards the driver
   * @param lookLeft      Face the robot left
   * @param lookRight     Face the robot right
   * @param lookTarget    Face the robot towards the vision target
   * @param hdgMode       Switch between angle and velocity mode
   */
    public AbsoluteDriveAdvHdg(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier oX, DoubleSupplier oY,
                              DoubleSupplier leftY, DoubleSupplier rightY, DoubleSupplier lookPOV, BooleanSupplier hdgMode)
    {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.oX = oX;
    this.oY = oY;
    this.leftY = leftY;
    this.rightY = rightY;
    this.lookPOV = lookPOV;
    this.hdgMode = hdgMode;
    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    resetHeading = true;
    angVelMode = true;
    hdgPOV = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute()
{
    double headingX = oX.getAsDouble();
    double headingY = oY.getAsDouble();



  // Convert joystick input to an angle
  double angle = Math.toDegrees(Math.atan2(headingY, headingX))-270;

  // Normalize to the range [0, 360)
  angle = (angle + 360) % 360;

  // Snap to the nearest 60-degree increment
  double snappedAngle = Math.round(angle / 60.0) * 60.0;
  if (headingX != 0 || headingY != 0) {
    headingX = Math.sin(Math.toRadians(-snappedAngle));
    headingY = Math.cos(Math.toRadians(-snappedAngle));

    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isPresent()) {
      if (allianceColor.get() == Alliance.Red) {
        if(snappedAngle ==   0.0){AprilTagConstants.ReefTagID = 7 ;};
        if(snappedAngle ==  60.0){AprilTagConstants.ReefTagID = 8 ;};
        if(snappedAngle == 120.0){AprilTagConstants.ReefTagID = 9 ;};
        if(snappedAngle == 180.0){AprilTagConstants.ReefTagID = 10;};
        if(snappedAngle == 240.0){AprilTagConstants.ReefTagID = 11;};
        if(snappedAngle == 300.0){AprilTagConstants.ReefTagID = 6 ;};
      }
      else if (allianceColor.get() == Alliance.Blue) {
        if(snappedAngle ==   0.0){AprilTagConstants.ReefTagID = 18;};
        if(snappedAngle ==  60.0){AprilTagConstants.ReefTagID = 19;};
        if(snappedAngle == 120.0){AprilTagConstants.ReefTagID = 20;};
        if(snappedAngle == 180.0){AprilTagConstants.ReefTagID = 21;};
        if(snappedAngle == 240.0){AprilTagConstants.ReefTagID = 22;};
        if(snappedAngle == 300.0){AprilTagConstants.ReefTagID = 17;};
      }
    }
  } else {
    Rotation2d currentHeading = swerve.getHeading(); 
    headingX = currentHeading.getSin();
    headingY = currentHeading.getCos();
  }
  
  // System.out.println("Snapped Angle: " + snappedAngle);

    if (hdgMode.getAsBoolean() && !hdgModePressed) {
        hdgModePressed = true; // Button pressed, set flag to true

        if (angHdgMode) {
            angVelMode = true; // Switch to velocity mode
            angHdgMode = false; // Switch off angle mode
        } else {
            angHdgMode = true; // Switch to angle mode
            angVelMode = false; // Switch off velocity mode
            // Prevent the robot from spinning when switching modes
            Rotation2d currentHeading = swerve.getHeading();
            headingX = currentHeading.getSin();
            headingY = currentHeading.getCos();
        }
    } else if (!hdgMode.getAsBoolean()) {
        hdgModePressed = false; // Button released, reset flag
    }

    if (lookPOV.getAsDouble() != -1) // If the POV is not in the center
    {
        headingX = Rotation2d.fromDegrees(-lookPOV.getAsDouble()).getSin(); // Get the x component of the angle
        headingY = Rotation2d.fromDegrees(-lookPOV.getAsDouble()).getCos(); // Get the y component of the angle
        hdgPOV = true; // Set the flag to true
    }



    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), -vY.getAsDouble(), headingX, headingY); // Get the desired chassis speeds based on a 2 joystick module.

    Translation2d translationY = 
    new Translation2d(0, leftY.getAsDouble() - rightY.getAsDouble()); // Get the translation for the y-axis
    // Limit velocity to prevent tipping
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                          Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                                          swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    if (angHdgMode)
    {
      if (translationY.getY() != 0) {
        swerve.drive(translationY, 0, false);
      } else {
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
      }
    }
    if (angVelMode)
    {
      if (translationY.getY() != 0) {
        swerve.drive(translationY, 0, false);
      } else {
        swerve.drive(translation, MathUtil.applyDeadband(-oX.getAsDouble() * 5, OperatorConstants.RIGHT_X_DEADBAND), true);
      }
    }
}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }

}

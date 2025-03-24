// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AprilTagConstants;
import java.util.Optional;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    setAprilTag();
    m_robotContainer.setMotorBrake(true);
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
    //m_robotContainer.elevatorSubsystem.ElevatorInitCmd().schedule();
    //m_robotContainer.sliderSubsystem.SliderInitCmd().schedule();

    m_robotContainer.climberSubsystem.climbAndGetPaid(180.0).schedule();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }

    setAprilTag();
    m_robotContainer.rotateSubsystem.setRotateAngle(Constants.CoralConstants.CORAL_OFF_ELEVATOR);
<<<<<<< Updated upstream
    m_robotContainer.sliderSubsystem.SliderInitCmd().schedule();
    m_robotContainer.elevatorSubsystem.ElevatorInitCmd().schedule();
    m_robotContainer.climberSubsystem.climbAndGetPaid(180.0).schedule();
=======
    if(!Robot.isSimulation()){
      m_robotContainer.sliderSubsystem.SliderInitCmd().schedule();
      m_robotContainer.elevatorSubsystem.ElevatorInitCmd().schedule();
    }

>>>>>>> Stashed changes
    // m_robotContainer.climberSubsystem.releaseClimber(true);

    if (Robot.isSimulation()){
      Optional<Alliance> allianceColor = DriverStation.getAlliance();
      if (allianceColor.isPresent()) {
        if (allianceColor.get() == Alliance.Red) {
          m_robotContainer.drivebase.resetOdometry(new Pose2d(10, 2.5, new Rotation2d(0)));
        }
        else{
          m_robotContainer.drivebase.resetOdometry(new Pose2d(7.5, 5.5, new Rotation2d(Math.PI)));
        }}}
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {

    if(m_robotContainer.sliderSubsystem.armSliderLimitSwitch.isPressed()){
      m_robotContainer.sliderSubsystem.coralSldrEnc.setPosition(0);
    }

    if(!m_robotContainer.elevatorSubsystem.limitSw.get()){
      m_robotContainer.elevatorSubsystem.elevEncFlw.setPosition(0);
      m_robotContainer.elevatorSubsystem.elevEncLdr.setPosition(0);
    }
       
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
    // m_robotContainer.getSnappedAngleID();
    // SmartDashboard.putNumber("Reef Tag ID", AprilTagConstants.ReefTagID);
    // System.out.println("Reef Tag ID: " + AprilTagConstants.ReefTagID);
  }
  
  void setAprilTag()
  {
    Optional<Alliance> allianceColor = DriverStation.getAlliance();
    if (allianceColor.isPresent()) {
      if (allianceColor.get() == Alliance.Red) {
        AprilTagConstants.ReefTagID = 7;
        AprilTagConstants.Reef0   = 7 ;
        AprilTagConstants.Reef60  = 8 ;
        AprilTagConstants.Reef120 = 9 ;
        AprilTagConstants.Reef180 = 10;
        AprilTagConstants.Reef240 = 11;
        AprilTagConstants.Reef300 = 6 ;
        AprilTagConstants.HumanPlayerLeft = 1;
        AprilTagConstants.HumanPlayerRight = 2;
        AprilTagConstants.Processor = 3;
        AprilTagConstants.BargeFront = 5;
        AprilTagConstants.BargeBack = 15;
      }
      else if (allianceColor.get() == Alliance.Blue) {
        AprilTagConstants.ReefTagID = 18;
        AprilTagConstants.Reef0    = 18;
        AprilTagConstants.Reef60   = 19;
        AprilTagConstants.Reef120  = 20;
        AprilTagConstants.Reef180  = 21; 
        AprilTagConstants.Reef240  = 22;
        AprilTagConstants.Reef300  = 17;
        AprilTagConstants.HumanPlayerLeft = 13;
        AprilTagConstants.HumanPlayerRight = 12;
        AprilTagConstants.Processor = 16;
        AprilTagConstants.BargeFront = 14;
        AprilTagConstants.BargeBack = 4;
      }
    }
    }
}

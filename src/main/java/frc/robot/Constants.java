// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static double Max_Speed_Multiplier = 0.75;
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class ElevatorConstants {
    public static final int LEFT_ELEVATOR_MOTOR_PORT = 9;
    public static final int RIGHT_ELEVATOR_MOTOR_PORT = 13;

    public static final double START_POSE = 0;
    public static final double TROUGH_POSE = 1;
    public static final double REEF_LOW_POSE = 0.5;
    public static final double REEF_MIDDLE_POSE = 6.125;
    public static final double REEF_HIGH_POSE = 14.9; //14.75
    public static final double ALGAE_SCORE_POSE = 0;
    public static final double ALGAE_PICKUP_HIGH_POSE = 6.125;
    public static final double ALGAE_PICKUP_LOW_POSE = 0;
    public static final double HUMAN_PLAYER_POSE = 0;

  }

  public static class ArmConstants {
    public static final int ARM_MOTOR_PORT = 12;
    public static final int CORAL_INTAKE_POS = 190;
    public static final int CORAL_MID_POS = 165;
    public static final int CORAL_HIGH_POS = 150;
    public static final int ALGAE_INTAKE_POS = 240;
    public static final int ALGAE_START_POS = 280;
  }

  // public static class ServoConstants {

  //   public static final int PINCHER_SERVO_PORT_1 = 1;
  //   public static final int PINCHER_SERVO_PORT_2 = 2;
  //   public static final int CLOSED_ANGLE = 90;
  //   public static final int OPEN_ANGLE = 0;

  // }

  public static class IntakeConstants {
    public static int LEFT_INTAKE_MOTOR_PORT = 10;
    public static int RIGHT_INTAKE_MOTOR_PORT = 11;

    public static double INTAKE_SPEED = 600;
    public static double OUTTAKE_SPEED = -500;
    public static double STOP_SPEED = 0;
    public static double HOLD_SPEED = 200;
  }

  public static class AprilTagConstants {
    public static int HumanPlayerLeft = 0; // 1 red, 13 blue
    public static int HumanPlayerRight = 0; // 2 red, 12 blue
    public static int Processor = 0; // 3 red, 11 blue
    public static int BargeFront = 0; // 5, 14 both blue and red are on the same side of the field
    public static int BargeBack = 0; // 15, 4 both blue and red are on the same side of the field
    public static int Reef0 = 0; // 7 red, 18 Blue
    public static int Reef60 = 0; // 8 red, 17 Blue
    public static int Reef120 = 0; // 9 red, 22 Blue
    public static int Reef180 = 0; // 10 red, 21 Blue
    public static int Reef240 = 0; // 11 red, 20 Blue
    public static int Reef300 = 0; // 6 red, 19

    public static double SnappedHdg = -1.0; // Variable to store snapped heading
    public static int ReefTagID = 0; // Variable to store snapped angle

  }

}

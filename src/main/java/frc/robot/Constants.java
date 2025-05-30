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

  public static final double ROBOT_MASS = Units.lbsToKilograms(160);
  //TODO: Update the chassis weight and center of gravity location
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5); // Max free speed of the MK4C with Neo Vortex motors and L1 gearing is 16.6 ft/s
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
    public static double Max_Speed_Multiplier = 1.0;
  }

  public static class AutonConstants
  {
    public static final double LINEAR_VELOCITY = 2.0; //MPS
    public static final double LINEAR_ACELERATION = 2.0; //Meters per second squared
    public static final double ANGULAR_VELOCITY = 720; //Degrees
    public static final double ANGULAR_ACCELERATION = 360; //Degrees per second squared
  
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
    public static final int RIGHT_ELEVATOR_MOTOR_PORT = 14;
    public static final int LEFT_ELEVATOR_MOTOR_PORT = 15;
    

    public static final double START_POSE = 0;
    public static final double TROUGH_POSE = 1;
    public static final double REEF_L2_POSE = 2.0; //3.0 before // 2.0 for laksehore
    public static final double REEF_L3_POSE = 10.0; // 11.0 = coral //10. = lakeshore
    public static final double REEF_L4_POSE = 23.5; //14.75
    public static final double ALGAE_BARGE_POSE = 24.0;
    public static final double ALGAE_PROCESSOR_POSE = 0; 
    public static final double ALGAE_PICKUP_HIGH_POSE = 10.0;
    public static final double ALGAE_PICKUP_LOW_POSE = 2.5;
    public static final double HUMAN_PLAYER_POSE = 0;

  }

  public static class AlgaeRotateConstants {
    public static final int ALGAE_INTAKE_POS = -15;
    public static final int ALGAE_START_POS = -0;
    public static final int ALGAE_BARGE_POS = -28;
    public static final int ALGAE_PROCESSOR_POS = 0;
    public static final int ALGAE_ROTATE_MOTOR_PORT = 20;
    // public static final int ALGAE_ADJUST_MOTOR_PORT = 0;
    // public static final int ALGAE_Intake_MOTOR_PORT = 0;
  }


  public static class AlgaeIntakeConstants {
    // public static int LAUNCHER_LEADER_MOTOR_PORT = 21;
    // public static int LAUNCHER_FOLLOWER_MOTOR_PORT = 22;
    public static int FEEDER_LEADER_MOTOR_PORT = 23;
    // public static int FEEDER_FOLLOWER_MOTOR_PORT = 24;
    
    public static double INTAKE_SPEED = 600;
    public static double OUTTAKE_SPEED = -500;
    public static double STOP_SPEED = 0;
    public static double HOLD_SPEED = 200;
  }

  public static class CoralConstants {
    public static int CORAL_ROTATE_MOTOR_PORT = 17;
    public static int CORAL_SLIDER_MOTOR_PORT = 18;
    // public static int CORAL_HOLD_MOTOR_PORT = 19;
    // public static int SERVO_PORT = 0;
    public static double CORAL_L4_ANGLE = 160; // was 287.5
    public static double CORAL_L2_L3_ANGLE = 155.0; //was 155.0
    // public static double CORAL_LOW_ANGLE = 155.0;
    public static double CORAL_OFF_ELEVATOR = 185.0; //was 195 changed to avoid breaking
    public static double CORAL_START_ANGLE = 185.0; //was 195 changed to avoid breaking
    public static double ALGAE_SCORE_ANGLE = 65.0;
    public static double ALGAE_INTAKE_ANGLE = 137.5;
    public static double CORAL_SLIDER_LEFT_POSITION = -10.5;
    public static double CORAL_SLIDER_MIDDLE_POSITION = -4.0;
    public static double CORAL_SLIDER_RIGHT_POSITION = -.5;
    public static int BEAM_BREAK_SENSOR_PORT = 0;

    public static double CLIMB_POS  = 140.0;
  }

  public static class OuttakeConstants {
    public static int INDEX_LDR_PORT = 21;
    public static int INDEX_FLW_PORT = 22;
  }

  public static class ClimbConstants {
    public static int CLIMBER_MOTOR_PORT = 16;
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
// SubSystem	Description               Device                      Network     ID	
// Power      Distribution Hub			    PDH                         Rio	        0			
// Swerve	    Front Left - Drive	        Vortex - SparkFlex	        CANivore	  1			
// Swerve	    Front Left - Angle	        Vortex - SparkFlex	        CANivore	  2			
// Swerve	    Front Left - Steer         Encoder	CANcoder	          CANivore	  3			
// Swerve	    Front Right - Drive        Vortex - SparkFlex	        CANivore	  4			
// Swerve	    Front Right - Angle        Vortex - SparkFlex	        CANivore	  5			
// Swerve	    Front Right - Steer        Encoder	CANcoder	          CANivore	  6			
// Swerve	    Back Right - Drive        Vortex - SparkFlex	        CANivore	  7			
// Swerve	    Back Right - Angle        Vortex - SparkFlex	        CANivore	  8			
// Swerve	    Back Right - Steer        Encoder	CANcoder	          CANivore	  9			
// Swerve	    Back Left - Drive     	Vortex - SparkFlex	        CANivore	  10			
// Swerve	    Back Left - Angle       Vortex - SparkFlex	        CANivore	  11			
// Swerve	    Back Left - Steer       Encoder	CANcoder	          CANivore	  12			
// Swerve	    IMU                     	Pegion 2	                  CANivore	  13			
// Elevator 	Motor Right (Ldr)	        Vortex - SparkFlex	        Rio	        14			
// Elevator 	Motor Left (Flwr)	        Vortex - SparkFlex	        Rio	        15			
// Elevator 		                        Limit Switch				        Connected to Right Elevator SparFlex
// Climber    Motor	                    Neo - SparkMax	            Rio	        16			
// Climber    Climb                     Encoder	Absolute Encoder    Connected to Climber SparkMax				
// Climber    Servo - Ratchet	          am-4954			                PWM         0		
// Coral	    Rotate	                  Neo - SparkMax	            Rio	        17			
// Coral	    Rotate	                  Absolute Encoder            SparkMax D  irect				
// Coral	    Traverse                  Neo550 - SparkMax	          Rio	        18			
// Coral	    Traverse                  Limit Switch					      Connected to Traverse SparkMax
// Coral	    Servo - Release	          Axon Max			              PWM         1		
// Coral	    Index	                    Neo550 - SparkMax	          Rio	        19			
// Coral	    	                        Beam Brake				          DIO         1	
// Coral	    	                        Proximity Sensor				    DIO         2	
// Algae	    Rotate	                  Vortex - SparkFlex	        Rio	        20			
// Algae	    Launch - Lower - Ldr	    Vortex - SparkFlex	        Rio	        21			
// Algae	    Launch - Upper - Flwr	    Vortex - SparkFlex	        Rio	        22			
// Algae	    Feeder - Lower - Ldr	    Neo550 - SparkMax	          Rio	        23			
// Algae	    Feeder - Upper - Flwr	    Neo550 - SparkMax	          Rio	        24
// LED        LED Strip	                Addressable LED Strip	      PWM	        2			


}   

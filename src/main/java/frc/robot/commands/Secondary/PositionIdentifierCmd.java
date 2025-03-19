// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import java.util.function.DoubleSupplier;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;

public class PositionIdentifierCmd extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final RotateSubsystem rotateSubsystem;
    private final DoubleSupplier  oX, oY;

    /**
     * Command to set the position of the elevator and rotate subsystems based on inputs a stick.
     * @param elevatorSubsystem The subsystem responsible for controlling the elevator mechanism.
     * @param rotateSubsystem The subsystem responsible for controlling the rotation mechanism.
     * @param intakeSubsystem The subsystem responsible for controlling the intake mechanism.
     * @param oX A DoubleSupplier providing the X coordinate of the input stick which will be rounded to 45 degree increments.
     * @param oY A DoubleSupplier providing the Y coordinate of the input stick which will be rounded to 45 degree increments.
     */
    public PositionIdentifierCmd(ElevatorSubsystem elevatorSubsystem, RotateSubsystem rotateSubsystem, DoubleSupplier oX, DoubleSupplier oY) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.rotateSubsystem = rotateSubsystem;
        this.oX = oX;
        this.oY = oY;
        addRequirements(elevatorSubsystem, rotateSubsystem);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // RobotContainer.getSnappedAngleID();
        // if(RobotContainer.right){
        //     goalPose = Vision.getAprilTagPose(AprilTagConstants.ReefTagID, new Transform2d(0.6604,   -.164338,
        //     Rotation2d.fromDegrees(180)));
        // } else{
        //     goalPose = Vision.getAprilTagPose(AprilTagConstants.ReefTagID, new Transform2d(0.6604,   .164338,
        //     Rotation2d.fromDegrees(180)));
        // }
        // sliderOffset = goalPose.getY();
        //sliderOffset = Math.sqrt(Math.pow(goalPose.getX() - drivebase.getPose().getX(), 2) + Math.pow(goalPose.getY() - drivebase.getPose().getY(), 2));
    }

    /**
     * Executes the command based on the joystick input.
     * 
     * This method reads the joystick's X and Y axis values, calculates the angle,
     * and snaps it to the nearest 45-degree increment. Depending on the snapped
     * angle, it schedules different sequences of commands for the robot's subsystems.
     * 
     * The possible snapped angles and their corresponding actions are:
     * - 315.0: Rotate to ALGAE_INTAKE_POS, run intake at 15% speed, then elevate to ALGAE_PICKUP_HIGH_POSE, rotate to CORAL_MID_POS, and run intake at 10% speed.
     * - 225.0: Rotate to ALGAE_INTAKE_POS, run intake at 15% speed, then elevate to ALGAE_PICKUP_LOW_POSE, rotate to CORAL_MID_POS, and run intake at 10% speed.
     * - 45.0: Elevate to REEF_HIGH_POSE, rotate to CORAL_HIGH_POS, elevate slightly more, rotate to ALGAE_INTAKE_POS, and elevate to HUMAN_PLAYER_POSE.
     * - 90.0: Elevate to REEF_MIDDLE_POSE, rotate to CORAL_MID_POS, elevate slightly more, rotate to ALGAE_INTAKE_POS, and elevate to HUMAN_PLAYER_POSE.
     * - 135.0: Elevate to REEF_LOW_POSE, rotate slightly more, elevate slightly more, rotate to ALGAE_INTAKE_POS, and elevate to HUMAN_PLAYER_POSE.
     * 
     * If the joystick is not pushed beyond a threshold, no action is taken.
     */
    @Override
    public void execute() {
        //System.out.println("Slider Offset: " + sliderOffset);
        // boolean inputAngleBol = false; // flag to track if the joystick is pushed
        double snappedInputAngle = -1.0; // initialize snappedInputAngle variable
        double oXRaw = oX.getAsDouble(); // get the joystick X axis values
        double oYRaw = oY.getAsDouble(); // get the joystick Y axis values
    
        
        // if (Math.abs(oXRaw) > 0.1 || Math.abs(oYRaw) > 0.1) {
        if(Math.sqrt(Math.pow(oXRaw,2) + Math.pow(oYRaw, 2)) > 0.1) {
            // inputAngleBol = true; // if the joystick is pushed
            double inputAngle = Math.toDegrees(Math.atan2(oYRaw, oXRaw)) - 270; // -270 to make 0 degrees straight up
            inputAngle = (inputAngle + 360) % 360; // 360 degrees in a circle
            snappedInputAngle = Math.round(inputAngle / 45) * 45.0; // 45 degree increments
            snappedInputAngle = (snappedInputAngle + 360) % 360; // normalize to 0-360
        }

        SmartDashboard.putNumber("Engineer Snapped Angle", snappedInputAngle); // display the snapped angle on the SmartDashboard

        if (snappedInputAngle == 270) { //if the joystick is pushed up and to the left
            Commands.sequence(
                Commands.parallel(
                    elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_L2_POSE), 
                    rotateSubsystem.setRotateAngleCmd(CoralConstants.CORAL_L2_L3_ANGLE)
                ))
                .schedule();
            
        } else if (snappedInputAngle == 90) { 
            Commands.sequence(
                Commands.parallel(
                    elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_L4_POSE), 
                    rotateSubsystem.setRotateAngleCmd(CoralConstants.CORAL_L4_ANGLE)
                ))
                .schedule();
       
        } else if (snappedInputAngle == 0) { 
            Commands.sequence(
                Commands.parallel(
                    elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_L3_POSE), 
                    rotateSubsystem.setRotateAngleCmd(CoralConstants.CORAL_L2_L3_ANGLE)
                ))
                .schedule();
            } else if (snappedInputAngle == 180) {
                Commands.sequence( 
                    rotateSubsystem.setRotateAngleCmd(CoralConstants.CORAL_START_ANGLE),
                    elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.START_POSE)
                    )
                    .schedule();
           
            
        // } else if (snappedInputAngle == 45.0) {
        //     Commands.sequence(
        //         Commands.parallel(
        //             elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_HIGH_POSE), 
        //             // coralSubsystem.setSliderPosition(CoralConstants.CORAL_SLIDER_RIGHT_POSITION),
        //             coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_HIGH_ANGLE)
        //         )
        //         .andThen(
        //             coralSubsystem.OuttakeCmd()))
        //         .schedule();
        
        // } else if (snappedInputAngle == 90.0) {
        //     Commands.sequence(
        //         Commands.parallel(
        //             elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_MIDDLE_POSE), 
        //             // coralSubsystem.setSliderPosition(CoralConstants.CORAL_SLIDER_RIGHT_POSITION),
        //             coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_LOW_ANGLE)
        //         )
        //         .andThen(
        //             coralSubsystem.OuttakeCmd()))
        //         .schedule();
        
        // } else if (snappedInputAngle == 135.0) {
        //     Commands.sequence(
        //         Commands.parallel(
        //             elevatorSubsystem.ElevatorHeightCmd(ElevatorConstants.REEF_LOW_POSE+0.5), 
        //             // coralSubsystem.setSliderPosition(CoralConstants.CORAL_SLIDER_RIGHT_POSITION),
        //             coralSubsystem.setRotateAngleCmd(CoralConstants.CORAL_HIGH_ANGLE)
        //         )
        //         .andThen(
        //             coralSubsystem.OuttakeCmd()))
        //         .schedule();
        // }
    

        // elevatorSubsystem.setElevatorHeight(pose);
        // if(Math.abs(pose - elevatorSubsystem.elevEncLdr.getPosition()) <= 0.125){
        //     rotateSubsystem.setArm(rotatePose);
        // }
    }}
}

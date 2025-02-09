// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.IntakeSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;

public class PositionIdentifierCmd extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final RotateSubsystem rotateSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DoubleSupplier  oX, oY;

    /**
     * Command to set the position of the elevator and rotate subsystems based on inputs a stick.
     * @param elevatorSubsystem The subsystem responsible for controlling the elevator mechanism.
     * @param rotateSubsystem The subsystem responsible for controlling the rotation mechanism.
     * @param intakeSubsystem The subsystem responsible for controlling the intake mechanism.
     * @param oX A DoubleSupplier providing the X coordinate of the input stick which will be rounded to 45 degree increments.
     * @param oY A DoubleSupplier providing the Y coordinate of the input stick which will be rounded to 45 degree increments.
     */
    public PositionIdentifierCmd(ElevatorSubsystem elevatorSubsystem, RotateSubsystem rotateSubsystem, IntakeSubsystem intakeSubsystem, DoubleSupplier oX, DoubleSupplier oY){
        
        this.elevatorSubsystem = elevatorSubsystem;
        this.rotateSubsystem = rotateSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.oX = oX;
        this.oY = oY;
        addRequirements(elevatorSubsystem, rotateSubsystem, intakeSubsystem);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

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
        //System.out.println("Snapped Angle: " + snappedInputAngle);
        SmartDashboard.putNumber("Engineer Snapped Angle", snappedInputAngle); // display the snapped angle on the SmartDashboard
    
        if (snappedInputAngle == 315.0) { //if the joystick is pushed up and to the left
            Commands.race(
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.ALGAE_INTAKE_POS), // 240 degrees
                intakeSubsystem.RunIntakeCmd(0.15)) // 15% speed of ~5600 RPM
            .andThen(
            Commands.sequence(
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE), // 6.125 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_MID_POS), // 165 degrees
                intakeSubsystem.RunIntakeCmd(0.1))) // 10% speed of ~5600 RPM
            .schedule();
            // pose = Constants.ElevatorConstants.ALGAE_PICKUP_HIGH_POSE;
            // rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
        } else if (snappedInputAngle == 225.0) { //if the joystick is pushed left
            Commands.race(
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.ALGAE_INTAKE_POS), // 240 degrees
                intakeSubsystem.RunIntakeCmd(0.15)) // 15% speed of ~5600 RPM
            .andThen(
            Commands.sequence(
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE), // 0.0 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_MID_POS), // 165 degrees
                intakeSubsystem.RunIntakeCmd(0.1))) // 10% speed of ~5600 RPM
            .schedule();
            // pose = Constants.ElevatorConstants.ALGAE_PICKUP_LOW_POSE;
            // rotatePose = Constants.ArmConstants.ALGAE_INTAKE_POS;
// TODO: how are we going to access these in autonomous?
// TODO: add a sequence to score the algae in the processor when the stick is at 180 degrees
// TODO: add a sequence to score the algae in the processor when the stick is at 180 degrees
        } else if (snappedInputAngle == 45.0) { //if the joystick is pushed up and to the right
            Commands.sequence(
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_HIGH_POSE), // 14.9 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_HIGH_POS), // 150 degrees
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_HIGH_POSE + 0.5), // 15.4 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.ALGAE_INTAKE_POS), // 240 degrees
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.HUMAN_PLAYER_POSE)) // 0.0 inches
            .schedule();
            // pose = Constants.ElevatorConstants.REEF_HIGH_POSE;
            // rotatePose = Constants.ArmConstants.CORAL_HIGH_POS;
        } else if (snappedInputAngle == 90.0) { //if the joystick is in the middle
            Commands.sequence(
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_MIDDLE_POSE), // 6.125 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_MID_POS), // 165 degrees
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_MIDDLE_POSE + 0.5), // 6.625 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.ALGAE_INTAKE_POS), // 240 degrees
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.HUMAN_PLAYER_POSE)) // 0.0 inches
            .schedule();
            // pose = Constants.ElevatorConstants.REEF_MIDDLE_POSE;
            // rotatePose = Constants.ArmConstants.CORAL_MID_POS;
        } else if (snappedInputAngle == 135.0) { //if the joystick is pushed down
            Commands.sequence(
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_LOW_POSE), // 0.5 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.CORAL_MID_POS + 7.5), // 172.5 degrees
               elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.REEF_LOW_POSE + 0.5), // 1.0 inches
                rotateSubsystem.RotatePosCmd(Constants.ArmConstants.ALGAE_INTAKE_POS), // 240 degrees
                elevatorSubsystem.ElevatorHeightCmd(Constants.ElevatorConstants.HUMAN_PLAYER_POSE)) // 0.0 inches
            .schedule();
            // pose = Constants.ElevatorConstants.REEF_LOW_POSE;
            // rotatePose = Constants.ArmConstants.CORAL_MID_POS;
        }
    

        // elevatorSubsystem.setElevatorHeight(pose);
        // if(Math.abs(pose - elevatorSubsystem.elevEncLdr.getPosition()) <= 0.125){
        //     rotateSubsystem.setArm(rotatePose);
        // }
    }
}

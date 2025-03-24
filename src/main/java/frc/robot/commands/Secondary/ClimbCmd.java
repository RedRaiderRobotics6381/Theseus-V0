// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Secondary.ClimberSubsystem;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;
import frc.robot.subsystems.Secondary.SliderSubsystem;

public class ClimbCmd extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final RotateSubsystem rotateSubsystem;
    private final SliderSubsystem sliderSubsystem;

    /**
     * Command to set the position of the elevator and rotate subsystems based on inputs a stick.
     * @param climberSubsystem The subsystem responsible for controlling the climber mechanism.
     * @param rotateSubsystem The subsystem responsible for controlling the rotation mechanism.
     */
    public ClimbCmd(ClimberSubsystem climberSubsystem, RotateSubsystem rotateSubsystem, SliderSubsystem sliderSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.rotateSubsystem = rotateSubsystem;
        this.sliderSubsystem =sliderSubsystem;
        addRequirements(climberSubsystem, rotateSubsystem, sliderSubsystem);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        climberSubsystem.setClimbPosition(100.0);
        rotateSubsystem.setRotateAngle(Constants.CoralConstants.CLIMB_POS);
        sliderSubsystem.setSliderPosition(0);

    
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.CoralSubsystem;



public class SliderInitCmd extends Command {

  private final CoralSubsystem coralSubsystem;
  //private boolean climberInitialized = false;
  // private boolean climbed;
  // private boolean climbedL;
  // private boolean climbedR;
  private boolean sliderInitialized;
    
    
    public SliderInitCmd(CoralSubsystem coralSubsystem) {
      this.coralSubsystem = coralSubsystem;
    addRequirements(coralSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

// Called when the command is initially scheduled.
@Override
public void initialize() {
  sliderInitialized = false;
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  if(!coralSubsystem.coralLimitSwitch.isPressed()) {
    coralSubsystem.coralIndexMtr.set(-.125);
    //I think this can just be an else statement
  } else if(coralSubsystem.coralLimitSwitch.isPressed()) {
    coralSubsystem.coralIndexMtr.set(0);
  }
  
  if (coralSubsystem.coralLimitSwitch.isPressed()){
    sliderInitialized = true;
  }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  coralSubsystem.coralIndexMtr.set(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return sliderInitialized;
}
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Secondary;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Secondary.ElevatorSubsystem;


public class ElevatorInitCmd extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  //private boolean climberInitialized = false;
  // private boolean climbed;
  // private boolean climbedL;
  // private boolean climbedR;
  private boolean elevatorInitialized;
  
  public ElevatorInitCmd(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

// Called when the command is initially scheduled.
@Override
public void initialize() {
  elevatorInitialized = false;
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  // if(!elevatorSubsystem.limitSwL.get()){
  //   elevatorSubsystem.elevMtrLdr.set(-.125);
  //   //I think this can just be an else statement
  // } else if(elevatorSubsystem.limitSwL.get()) {
  //   elevatorSubsystem.elevMtrLdr.set(0);
  // }
  
  if (elevatorSubsystem.limitSwL.get()){
    elevatorInitialized = true;
  }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  elevatorSubsystem.elevMtrLdr.set(0);
  elevatorSubsystem.elevEncLdr.setPosition(0);
  elevatorSubsystem.elevEncFlw.setPosition(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return elevatorInitialized;
}
}

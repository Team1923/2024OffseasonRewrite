// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj2.command.Command;

/*
 * 
 * full eject
 * 
 * if desired intake roller speed is eject, do not actually let intake rollers move until the intake is deployed
 * intake arm must be deployed for roller speeds to roll at eject
 * 
 * If the intake is dropped and there is a note in the intake, then dont pull intake back up until note is fully situated within the shooter area (beam break 3 covered)
 * 
 * once 1 is covered after that do not lift the intake until 3 has been covered
 * 
 * 
 */











public class IntakeStateMachine extends Command {
  /** Creates a new IntakeStateMachine. */
  public IntakeStateMachine() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

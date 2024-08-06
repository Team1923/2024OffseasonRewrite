// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj2.command.Command;

public class FeederStateMachine extends Command {
  /** Creates a new FeederStateMachine. */
  public FeederStateMachine() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * MAIN FUNCTIONALITY REQUIRED:
     * - BACKING + FORWARD MOTION --> should be done in a switch statement's default case
     * - SCORING --> INWARD (scoring is now regardless of state which should just be making sure the arm + shooter is at state)
     * - EJECT --> same idea as scoring, but only validate the intake position
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.ShooterSubsystem;

public class BabyBirdCommand extends Command {

  StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new BabyBirdCommand. */
  public BabyBirdCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.desiredShooterState = ShooterSubsystem.States.BABY_BIRD_VELO;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stateHandler.desiredShooterState = ShooterSubsystem.States.IDLE_VELO;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !stateHandler.bb4Covered && stateHandler.bb3Covered;
  }
}

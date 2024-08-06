// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeRollerStates;

public class IntakeStateMachine extends Command {
  /** Creates a new IntakeStateMachine. */
  private IntakeSubsystem intakeSubsystem;

  private StateHandler stateHandler = StateHandler.getInstance();

  public IntakeStateMachine(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stateHandler.latchingBB = false; //TODO: resetting here good?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeRollerStates desiredRollerState = stateHandler.desiredIntakeRollerState;
    IntakeArmStates desiredArmState = stateHandler.desiredIntakeArmState;

    if (stateHandler.bb1Covered && desiredRollerState != IntakeRollerStates.EJECT) {
      stateHandler.latchingBB = true;
    }
  
      /* Boolean value changes based on BB3 and !BB1 */
    if (stateHandler.bb3Covered && !stateHandler.bb1Covered) {
      stateHandler.latchingBB = false;
    }

    switch(desiredArmState) {
      case STOWED:
      /* Latching boolean condition for bb1 making sure intake doesn't deploy until note is fully situated */
        if (!stateHandler.bb3Covered && stateHandler.latchingBB) {
          desiredArmState = IntakeArmStates.DEPLOYED; //TODO: not setting here
          desiredRollerState = IntakeRollerStates.INTAKE;
        } 

        /* Makes sure if arm is not deployed yet but want to eject, rollers don't spin */
        if (stateHandler.currentIntakeArmState != IntakeArmStates.DEPLOYED & desiredRollerState == IntakeRollerStates.EJECT) {
          desiredRollerState = IntakeRollerStates.OFF;
        }
  
      default:
        intakeSubsystem.setIntakeArmTo(desiredArmState.intakePosition);
        if(intakeSubsystem.isAtIntakeArmPosition(desiredArmState)) {
          stateHandler.currentIntakeArmState = desiredArmState;
        }
        intakeSubsystem.setRollerSpeedTo(desiredRollerState.OUTPUT);
        if(intakeSubsystem.isAtRollerVelocity(desiredRollerState)) {
          stateHandler.currentIntakeRollerState = desiredRollerState;
        }
    }

    /*
     * KEY NOTES FOR FUNCTIONALITY:
     * - Otherwise, standard up down depending on state
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

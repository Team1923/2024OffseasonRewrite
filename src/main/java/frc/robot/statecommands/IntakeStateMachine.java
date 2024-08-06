// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.IntakeSubsystem.RollerStates;

public class IntakeStateMachine extends Command {
  /** Creates a new IntakeStateMachine. */
  private IntakeSubsystem intakesubsystem;
  private boolean bb1Crossed = false;

  private StateHandler stateHandler = StateHandler.getInstance();

  public IntakeStateMachine(IntakeSubsystem intakesubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakesubsystem = intakesubsystem;

    addRequirements(intakesubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    IntakeSubsystem.RollerStates desiredRollerState = stateHandler.desiredIntakeRollerState;
    IntakeSubsystem.IntakeArmStates desiredArmState = stateHandler.desiredIntakeArmState;

    if (stateHandler.bb1Covered && desiredRollerState != RollerStates.EJECT) {
      bb1Crossed = true;
    }
  
      /* Boolean value changes based on BB3 and !BB1 */
    if (stateHandler.bb3Covered && !stateHandler.bb1Covered) {
      bb1Crossed = false;
    }

    switch(desiredArmState) {
      case STOWED:
      /* Latching boolean condition for bb1 making sure intake doesn't deploy until note is fully situated */
        if (!stateHandler.bb3Covered && bb1Crossed) {
          intakesubsystem.setIntakeArmTo(IntakeArmStates.DEPLOYED.intakePosition);
          intakesubsystem.setRollerSpeedTo(RollerStates.INTAKE.OUTPUT);
        } 

        /* Makes sure if arm is not deployed yet but want to eject, rollers don't spin */
        if (desiredArmState != IntakeArmStates.DEPLOYED & desiredRollerState == RollerStates.EJECT) {
          intakesubsystem.setRollerSpeedTo(RollerStates.OFF.OUTPUT);
        }
      default:
        intakesubsystem.setIntakeArmTo(desiredArmState.intakePosition);
        if(intakesubsystem.isAtIntakeArmPosition(desiredArmState)) {
          stateHandler.currentIntakeArmState = stateHandler.desiredIntakeArmState;
        }
        intakesubsystem.setRollerSpeedTo(desiredRollerState.OUTPUT);
        if(intakesubsystem.isAtRollerVelocity(desiredRollerState)) {
          stateHandler.currentIntakeRollerState = stateHandler.desiredIntakeRollerState;
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

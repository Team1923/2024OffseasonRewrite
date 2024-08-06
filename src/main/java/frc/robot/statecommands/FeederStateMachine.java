// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class FeederStateMachine extends Command {

  private FeederSubsystem feederSubsystem;

  private StateHandler stateHandler = StateHandler.getInstance();


  //TODO: A timer for overriding the feeder stalling should probably live here

  /** Creates a new FeederStateMachine. */
  public FeederStateMachine(FeederSubsystem feederSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.feederSubsystem = feederSubsystem;

    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*TONS of gremlins in this logic */
    
    FeederSubsystem.States desiredState = stateHandler.desiredFeederState;

    if (desiredState == FeederSubsystem.States.FULL_EJECT){ //fall through if full eject
      
    }
    else if (stateHandler.currentIntakeState == IntakeSubsystem.ArmStates.DEPLOYED
          && stateHandler.desiredIntakeState == IntakeSubsystem.ArmStates.DEPLOYED
          && stateHandler.currentRollerSpeed == IntakeSubsystem.RollerStates.EJECT){ //ejecting (this really probably should be a restricting case instead of a setting case)
            desiredState = FeederSubsystem.States.OUTWARD;
    }
    /* About to go into all the shooting states, these should probably be done in a switch statement of ScoringTypes. Also I think backing should have higher precidence? They should also probably be restrictive of not being in current position rather than setting */
    else if (stateHandler.currentArmState == ArmSubsystem.States.SPEAKER 
          && stateHandler.currentShooterState == ShooterSubsystem.States.RANGED_VELO
          && ((stateHandler.isCenteredToSpeakerTag() && stateHandler.isInSpeakerRange()) || stateHandler.autoOverride)
          //&& timer good for arm movement, hopefully we don't need this, otherwise the timer should probably live as a restrictor in arm state machine to restrict current position?
          ){ //RANGED 
          desiredState = FeederSubsystem.States.INWARD;
    }
    else if (stateHandler.currentArmState == ArmSubsystem.States.TRAP
            && stateHandler.currentShooterState == ShooterSubsystem.States.TRAP_VELO
            && trap timer)


    feederSubsystem.setFeederTo(desiredState.OUTPUT);

    if (feederSubsystem.isAtState(desiredState)){
      stateHandler.currentFeederState = desiredState;
    }
    
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

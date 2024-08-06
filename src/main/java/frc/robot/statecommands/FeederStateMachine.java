// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;

public class FeederStateMachine extends Command {

  private FeederSubsystem feederSubsystem = new FeederSubsystem();
  
  private StateHandler stateHandler = StateHandler.getInstance();

  private Timer stallTimer;

  /** Creates a new FeederStateMachine. */
  public FeederStateMachine(FeederSubsystem feederSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.feederSubsystem = feederSubsystem;

    stallTimer = new Timer();

    addRequirements(feederSubsystem);
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
     * - SCORING --> INWARD (look at old code, see how many cases in which the feeder runs inward)
     * - EJECT --> same idea as scoring, but only validate the intake position
     */


     

    FeederSubsystem.States desiredState = stateHandler.desiredFeederState;


    if (desiredState != FeederSubsystem.States.FEED_TO_SHOOTER && stallTimer.get() != 0){
      stallTimer.stop();
      stallTimer.reset();
    }


    if (desiredState == FeederSubsystem.States.FULL_EJECT){

    }
    else if (desiredState == FeederSubsystem.States.FEED_TO_INTAKE
          && stateHandler.desiredIntakeArmState == IntakeSubsystem.IntakeArmStates.DEPLOYED
          && stateHandler.currentIntakeArmState != IntakeSubsystem.IntakeArmStates.DEPLOYED){
            desiredState = FeederSubsystem.States.OFF;
    }
    // else if (stateHandler.hasGamePiece() && stateHandler.bb4Covered){
    //   desiredState = FeederSubsystem.States.BACKING;
    // }
    else if (stateHandler.hasGamePiece() && desiredState == FeederSubsystem.States.FEED_TO_SHOOTER){ //this will technically activate while intaking, but shouldn't matter since no shooting command will run.  

      if (stallTimer.get() == 0){
        stallTimer.start();
      }

      if (!stallTimer.hasElapsed(FeederConstants.timeout)){
        switch(stateHandler.scoringType){
        case AMP:
          break;
        case SUBWOOFER:
          break;
        case REVERSE_SUBWOOFER:
          break;
        case RANGED:
          if ()
          break;
        case TRAP:
          break;
        case HIGH_PUNT:
          break;
        case LOW_PUNT:
          break;
        case UNGUARDABLE:
          break;
        case CLIMB:
          break;
        }
      }
      
      
    }
    else if (desiredState == FeederSubsystem.States.OFF
            && stateHandler.desiredArmState == ArmStates.STOWED
            && stateHandler.currentArmState == ArmStates.STOWED
            && stateHandler.desiredShooterState == ShooterSubsystem.States.IDLE_VELO){

            desiredState = FeederSubsystem.States.BACKING;
    }

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

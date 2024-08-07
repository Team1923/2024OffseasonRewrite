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
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

public class FeederStateMachine extends Command {

  private FeederSubsystem feederSubsystem;
  
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


     

    FeederStates desiredState = stateHandler.desiredFeederState;


    if (desiredState != FeederStates.FEED_TO_SHOOTER && stallTimer.get() != 0){
      stallTimer.stop();
      stallTimer.reset();
    }


    if (desiredState == FeederStates.FULL_EJECT){

    }
    else if (desiredState == FeederStates.FEED_TO_INTAKE
          && stateHandler.desiredIntakeArmState == IntakeArmStates.DEPLOYED
          && stateHandler.currentIntakeArmState != IntakeArmStates.DEPLOYED){
            desiredState = FeederStates.OFF;
    }
    else if (stateHandler.latchingBB){
      desiredState = FeederStates.FEED_TO_SHOOTER;
    }
    /* I think there may be merit to putting this here */
    // else if (stateHandler.hasGamePiece() && stateHandler.bb4Covered){
    //   desiredState = FeederStates.BACKING;
    // }
    else if (stateHandler.hasGamePiece() && desiredState == FeederStates.FEED_TO_SHOOTER){ //this will technically activate while intaking, but shouldn't matter since no shooting command will run.  

      if (stallTimer.get() == 0){
        stallTimer.start();
      }

      if (!stallTimer.hasElapsed(FeederConstants.timeout)){
        switch(stateHandler.scoringType){

        case AMP:
          if (stateHandler.currentArmState == ArmStates.AMP
            && stateHandler.currentShooterState == ShooterStates.FRONT_AMP_VELO){
              break;
          }

        case SUBWOOFER:
          if (stateHandler.currentArmState == ArmStates.SUBWOOFER
            && stateHandler.currentShooterState == ShooterStates.SUBWOOFER_VELO){
              break;
          }

        case REVERSE_SUBWOOFER:
          if (stateHandler.currentArmState == ArmStates.REVERSE_SUBWOOFER
            && stateHandler.currentShooterState == ShooterStates.REVERSE_SUBWOOFER_VELO){
              break;
          }

        case RANGED:
          // if (stateHandler.currentArmState == ArmStates.RANGED
          //   && stateHandler.currentShooterState == ShooterStates.RANGED_VELO
          //   && ( (stateHandler.isCenteredToSpeakerTag() && stateHandler.isInSpeakerRange()) || stateHandler.autoOverride())){
            if(false){
              break;
            }  
            // } 

        case TRAP:
          if (stateHandler.currentArmState == ArmStates.TRAP
            && stateHandler.currentShooterState == ShooterStates.TRAP_VELO){
              break;
            }

        case HIGH_PUNT:
          if (stateHandler.currentArmState == ArmStates.PUNT_HIGH
            && stateHandler.currentShooterState == ShooterStates.PUNT_HIGH_VELO){
              break;
            }

        case LOW_PUNT:
          if (stateHandler.currentArmState == ArmStates.PUNT_LOW
            && stateHandler.currentShooterState == ShooterStates.PUNT_LOW_DUTY){
              break;
            }

        case UNGUARDABLE:
          if (stateHandler.currentArmState == ArmStates.UNGUARDABLE
            && stateHandler.currentShooterState == ShooterStates.UNGUARDABLE_VELO){
              break;
            }

        case CLIMB: //falls through, shouldn't be feeding in climb
                  
        default:
          desiredState = FeederStates.OFF;
          }
      }
      
      
    }
    else if (desiredState == FeederStates.OFF
            && stateHandler.desiredArmState == ArmStates.STOWED
            && stateHandler.currentArmState == ArmStates.STOWED
            && stateHandler.desiredShooterState == ShooterStates.IDLE_VELO
            && stateHandler.bb4Covered){

            desiredState = FeederStates.BACKING;
    }

    feederSubsystem.setFeederTo(desiredState.REQUEST);

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.StateHandler.ScoringType;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.ShooterSubsystem.BlowerStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

public class ShootGamePiece extends Command {

  private StateHandler stateHandler;

  private Timer multipurposeTimer;

  /** Creates a new ShootGamePiece. */
  public ShootGamePiece() {
    // Use addRequirements() here to declare subsystem dependencies.

    multipurposeTimer = new Timer();
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(stateHandler.scoringType){
      case AMP:
        stateHandler.desiredArmState = ArmStates.AMP;
        stateHandler.desiredShooterState = ShooterStates.FRONT_AMP_VELO;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        stateHandler.blowerState = BlowerStates.ON;
        break;
      case CLIMB:
        System.out.println("TRYING TO SHOOT IN CLIMB MODE?");
        break;
      case HIGH_PUNT:
        stateHandler.desiredArmState = ArmStates.PUNT_HIGH;
        stateHandler.desiredShooterState = ShooterStates.PUNT_HIGH_VELO;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        break;
      case LOW_PUNT:
        stateHandler.desiredArmState = ArmStates.PUNT_LOW;
        stateHandler.desiredShooterState = ShooterStates.PUNT_LOW_DUTY;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        break;
      case RANGED:
        stateHandler.desiredArmState = ArmStates.RANGED;
        stateHandler.desiredShooterState = ShooterStates.RANGED_VELO;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        stateHandler.swerveState = SwerveStates.GOAL_CENTRIC;
        break;
      case REVERSE_SUBWOOFER:
        stateHandler.desiredArmState = ArmStates.REVERSE_SUBWOOFER;
        stateHandler.desiredShooterState = ShooterStates.REVERSE_SUBWOOFER_VELO;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        break;
      case SUBWOOFER:
        stateHandler.desiredArmState = ArmStates.SUBWOOFER;
        stateHandler.desiredShooterState = ShooterStates.SUBWOOFER_VELO;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        break;
      case TRAP:
        stateHandler.desiredArmState = ArmStates.TRAP;
        stateHandler.desiredShooterState = ShooterStates.TRAP_VELO;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        stateHandler.blowerState = BlowerStates.ON;
        break;
      case UNGUARDABLE:
        stateHandler.desiredArmState = ArmStates.UNGUARDABLE;
        stateHandler.desiredShooterState = ShooterStates.UNGUARDABLE_VELO;
        stateHandler.desiredFeederState = FeederStates.FEED_TO_SHOOTER;
        stateHandler.blowerState = BlowerStates.ON;
        break;
      default:
        break;

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(stateHandler.scoringType){
      case TRAP:
        if (!stateHandler.bb3Covered) multipurposeTimer.start(); //keeping trap door open for long enough after
        break;
      case AMP:
        if(!stateHandler.bb3Covered) multipurposeTimer.start();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    multipurposeTimer.stop();
    multipurposeTimer.reset();

    stateHandler.desiredArmState = ArmStates.STOWED;
    stateHandler.desiredFeederState = FeederStates.OFF;
    stateHandler.desiredShooterState = ShooterStates.IDLE_VELO;

    stateHandler.swerveState = SwerveStates.FIELD_CENTRIC;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch(stateHandler.scoringType){
      case TRAP:
        if (!multipurposeTimer.hasElapsed(7)) return false;
        break;
      case AMP: 
        if (!multipurposeTimer.hasElapsed(5)) return false;
        break;
    }

    return !stateHandler.bb2Covered && !stateHandler.bb3Covered && !stateHandler.bb4Covered;


  }
}
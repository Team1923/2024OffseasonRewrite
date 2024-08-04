// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StateCommands;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.ShooterSubsystem.States;

public class ShooterStateMachine extends Command {

  private ShooterSubsystem shooterSubsystem;

  private StateHandler stateHandler = StateHandler.getInstance();

  private Timer puntTimer = new Timer();

  /** Creates a new ShooterStateMachine. */
  public ShooterStateMachine(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ShooterSubsystem.States desiredState = stateHandler.desiredShooterState;

    switch(desiredState){
      
      case PUNT_SHOT_HIGH:
        if (puntTimer.get() == 0){
          puntTimer.start();
        }
        else if (puntTimer.hasElapsed(0.5)){
          stateHandler.currentShooterStates = ShooterSubsystem.States.PUNT_SHOT_HIGH;
        }
        break;

      case RANGED:
        //((MotionMagicVelocityVoltage)(States.RANGED.OUTPUT)).Velocity = updated value;

      default:
        puntTimer.stop();
        puntTimer.reset();
        if (shooterSubsystem.isAtState(desiredState)){
            stateHandler.currentShooterStates = desiredState;
        }
        break;
    }
  

    
    shooterSubsystem.setShooterMotorsTo(desiredState.OUTPUT_TOP, desiredState.OUTPUT_BOTTOM);
    shooterSubsystem.setBlowerTo(stateHandler.blowerPercent);
    

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

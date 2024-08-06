// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;

public class ArmStateMachine extends Command {

  private ArmSubsystem armSubsystem;
  private Timer timer;

  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ArmStateMachine. */

  public ArmStateMachine(ArmSubsystem armSubsystem) {

    this.armSubsystem = armSubsystem;
    timer = new Timer();
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    ArmSubsystem.ArmStates desiredArmState = stateHandler.desiredArmState;


    // case RANGED_VELO:
    //     //((MotionMagicVelocityVoltage)(States.RANGED.OUTPUT)).Velocity = updated value;

    if (!armSubsystem.isAtState(desiredArmState)) {
      timer.restart();
    }

    if (timer.hasElapsed(desiredArmState.settleTime) && armSubsystem.isAtState(desiredArmState)) {
      stateHandler.currentArmState = desiredArmState;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}

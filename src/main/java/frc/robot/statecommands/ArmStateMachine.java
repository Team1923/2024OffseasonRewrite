// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;

public class ArmStateMachine extends Command {

  private ArmSubsystem armSubsystem;
  private Timer timer;
  private StateHandler stateHandler = StateHandler.getInstance();

  /** Creates a new ArmStateMachine. */

  private boolean armZeroed = false;
  private int zeroingCounter = 0;

  public ArmStateMachine(ArmSubsystem armSubsystem) {

    this.armSubsystem = armSubsystem;
    timer = new Timer();
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.stop();
    timer.reset();

    armZeroed = false;
  }

  @Override
  public void execute() {

    //add arm self zeroing

    ArmStates desiredArmState = stateHandler.desiredArmState;

    switch(desiredArmState){
    

      case RANGED: //Update the ranged shot's motion magic value if we want ot shoot ranged
        if (stateHandler.speakerDistance() != -1){ //if we can update the angle, do it, otherwise stay at the last ranged angle
            ((MotionMagicVoltage)(ArmStates.RANGED.REQUEST)).Position = InterpolationConstants.distanceToAngle.get(stateHandler.speakerDistance()) * ArmConstants.armDegreesToRots;
        }
    }


    // if (desiredArmState == ArmStates.STOWED && armSubsystem.getArmPositionDegrees() >= -0.1/*Math.abs(armSubsystem.getArmSupplyCurrent()) >= ArmConstants.armSupplyToZero*/){
    //   armSubsystem.zeroArm();
    //   // zeroingCounter = 0;
    //   armZeroed = true;
    // }
    // else if (desiredArmState == ArmStates.STOWED && (stateHandler.currentArmState == ArmStates.STOWED ||  stateHandler.currentArmState == ArmStates.ZEROING) && !armZeroed){
    //     desiredArmState = ArmStates.ZEROING;
    //     // zeroingCounter++;
    // }
    // else if (desiredArmState != ArmStates.STOWED || stateHandler.currentArmState != ArmStates.STOWED){
    //   armZeroed = false;
    // }
   
    

    if (desiredArmState != stateHandler.currentArmState && armSubsystem.isAtState(desiredArmState) && timer.get() == 0){ //Start the timer for the arm to settle when the arm motors are at the desired state (settle timer starts AFTER the motors are in the right place)
      timer.restart();
    }


    // if (!armSubsystem.isAtState(desiredArmState)) { //OLD WAY
    //   System.out.println("HERE");
    //   timer.restart();
    // }
    // SmartDashboard.putNumber("TIMER", timer.get());

    //Check for current position; arm motors must read that they are at position AND settle timer must have elapsed
    if (timer.hasElapsed(desiredArmState.settleTime) && armSubsystem.isAtState(desiredArmState)) {
      stateHandler.currentArmState = desiredArmState;
      timer.stop();
      timer.reset();
    }


    armSubsystem.setArmTo(desiredArmState.REQUEST);
    SmartDashboard.putString("request name", desiredArmState.name());
    

    SmartDashboard.putBoolean("ISZEROED", armZeroed);
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

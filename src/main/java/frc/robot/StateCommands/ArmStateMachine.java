// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.StateCommands;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.ShooterSubsystem.States;

public class ArmStateMachine extends Command {

    private ArmSubsystem armSubsystem;

    private StateHandler stateHandler = StateHandler.getInstance();

    /** Creates a new ShooterStateMachine. */
    public ArmStateMachine(ArmSubsystem armSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateHandler.ScoringType;
import frc.robot.Commands.Intake.BabyBirdCommand;
import frc.robot.Commands.Intake.FullEjectCommand;
import frc.robot.StateCommands.ShooterStateMachine;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {

  StateHandler stateHandler = StateHandler.getInstance();

  /* Controller Instantiations */
  private final CommandXboxController driverXboxController = new CommandXboxController(0);
  private final CommandPS5Controller operatorPS5Controller = new CommandPS5Controller(1);


  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /* Default commands */
    shooterSubsystem.setDefaultCommand(new ShooterStateMachine(shooterSubsystem));


    /* Driver Button Bindings */


    /* Operator Button Bindings */
    operatorPS5Controller.triangle().onTrue(scoringMode(ScoringType.RANGED));
    operatorPS5Controller.square().onTrue(scoringMode(ScoringType.AMP));
    operatorPS5Controller.cross().onTrue(scoringMode(ScoringType.SUBWOOFER));
    operatorPS5Controller.circle().onTrue(scoringMode(ScoringType.REVERSE_SUBWOOFER) );
    operatorPS5Controller.povUp().onTrue(scoringMode(ScoringType.TRAP));
    operatorPS5Controller.L2().onTrue(scoringMode(ScoringType.LOW_PUNT));
    operatorPS5Controller.R2().onTrue(scoringMode(ScoringType.HIGH_PUNT));

    operatorPS5Controller.create().whileTrue(new FullEjectCommand());

    operatorPS5Controller.povLeft().whileTrue(new BabyBirdCommand());






  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


  public Command scoringMode(ScoringType scoringType){
    return new InstantCommand( () -> stateHandler.scoringType = scoringType);
  }
}

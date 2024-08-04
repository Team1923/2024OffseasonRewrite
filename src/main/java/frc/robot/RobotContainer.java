// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateHandler.ScoringType;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {

  StateHandler stateHandler = StateHandler.getInstance();

  /* Controller Instantiations */
  private final CommandXboxController driverXboxController = new CommandXboxController(0);
  private final CommandPS4Controller operatorPS4Controller = new CommandPS4Controller(1);


  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    /* Operator Button Bindings */
    operatorPS4Controller.triangle().onTrue(scoringMode(ScoringType.RANGED) );
    operatorPS4Controller.square().onTrue(scoringMode(ScoringType.AMP) );
    operatorPS4Controller.cross().onTrue(scoringMode(ScoringType.SUBWOOFER) );
    operatorPS4Controller.circle().onTrue(scoringMode(ScoringType.REVERSE_SUBWOOFER) );
    operatorPS4Controller.povUp().onTrue(scoringMode(ScoringType.TRAP) );

    operatorPS4Controller.R2().whileTrue(scoringMode(ScoringType.TRAP));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }


  public Command scoringMode(ScoringType scoringType){
    return new InstantCommand( () -> stateHandler.scoringType = scoringType);
  }
}

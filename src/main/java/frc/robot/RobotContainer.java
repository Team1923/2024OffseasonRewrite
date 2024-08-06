// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.StateHandler.ScoringType;
import frc.robot.commands.intake.BabyBirdCommand;
import frc.robot.commands.intake.FullEjectCommand;
import frc.robot.lib.swerve.Telemetry;
import frc.robot.lib.swerve.TunerConstants;
import frc.robot.statecommands.ArmStateMachine;
import frc.robot.statecommands.FeederStateMachine;
import frc.robot.statecommands.IntakeStateMachine;
import frc.robot.statecommands.ShooterStateMachine;
import frc.robot.statecommands.SwerveStateMachine;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.InfoSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  StateHandler stateHandler = StateHandler.getInstance();

  /* Controller Instantiations */
  private final CommandXboxController driverXboxController = new CommandXboxController(0);
  private final CommandPS5Controller operatorPS5Controller = new CommandPS5Controller(1);

  /* Subsystem Instantiations */
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final FeederSubsystem feederSubsystem = new FeederSubsystem();
  private final SwerveSubsystem swerveSubsystem = TunerConstants.DriveTrain;

  /* Helper class Instantiation */
  private final InfoSubsystem infoSubsystem = new InfoSubsystem(driverXboxController, operatorPS5Controller);
  private final Telemetry swerveLogger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    /* Default commands */
    shooterSubsystem.setDefaultCommand(new ShooterStateMachine(shooterSubsystem));
    intakeSubsystem.setDefaultCommand(new IntakeStateMachine(intakeSubsystem));
    armSubsystem.setDefaultCommand(new ArmStateMachine(armSubsystem));
    feederSubsystem.setDefaultCommand(new FeederStateMachine(feederSubsystem));
    swerveSubsystem.setDefaultCommand(new SwerveStateMachine(swerveSubsystem, 
                                      () -> driverXboxController.getLeftY(),
                                      () -> driverXboxController.getLeftX(), 
                                      () -> driverXboxController.getRightX()));

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

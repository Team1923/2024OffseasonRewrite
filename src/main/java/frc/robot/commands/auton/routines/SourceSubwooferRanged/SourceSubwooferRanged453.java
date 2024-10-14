// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.SourceSubwooferRanged;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.StateHandler.ScoringType;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.scoring.ShootGamePiece;
import frc.robot.lib.autonutils.PathPlannerHelpers;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceSubwooferRanged453 extends SequentialCommandGroup {
  /** Creates a new SourceSubwooferRanged453. */
  public SourceSubwooferRanged453() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      RobotContainer.scoringMode(ScoringType.SUBWOOFER), 
      new ShootGamePiece(),
      RobotContainer.scoringMode(ScoringType.RANGED),

      //4 Note
      new ParallelDeadlineGroup(
        new PathPlannerAuto("SourceSubwooferStartTo4"),
        new DeployIntakeCommand()),

      // Shoot
      PathPlannerHelpers.commandPathFrom("StageRangedTo4"),
      new ShootGamePiece(),

      //5 Note
      new ParallelDeadlineGroup(
        PathPlannerHelpers.commandPathFrom("StageRangedTo5"),
        new DeployIntakeCommand()
      ),
      PathPlannerHelpers.commandPathFrom("5ToStagedRanged"),
      new ShootGamePiece(),

      //3Note
      new ParallelDeadlineGroup(
        PathPlannerHelpers.commandPathFrom("StageRangedTo3"), 
        new DeployIntakeCommand()
      ),
      PathPlannerHelpers.commandPathFrom("3ToStageRanged"),
      new ShootGamePiece()


    );
  }
}

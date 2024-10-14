// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton.routines.SubwooferSub;

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
public class SubwooferSubSMA extends SequentialCommandGroup {
    /** Creates a new SubwooferRangedSMA12. */
    public SubwooferSubSMA() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                // Preload
                RobotContainer.scoringMode(ScoringType.SUBWOOFER),
                new ShootGamePiece(),
                // Stage
                new ParallelDeadlineGroup(
                        new PathPlannerAuto("StartSubwooferSubS"),
                        new DeployIntakeCommand()),

                PathPlannerHelpers.commandPathFrom("StageToSubwooferStart"),
                new ShootGamePiece(),

                // Middle
                new ParallelDeadlineGroup(
                        PathPlannerHelpers.commandPathFrom("SubwooferStartToMiddle"),
                        new DeployIntakeCommand()),

                PathPlannerHelpers.commandPathFrom("MiddleToSubwooferStart"),
                new ShootGamePiece(),

                // Amp
                new ParallelDeadlineGroup(
                        PathPlannerHelpers.commandPathFrom("SubwooferStartToAmp"),
                        new DeployIntakeCommand()),

                PathPlannerHelpers.commandPathFrom("AmpToSubwooferStart"),
                new ShootGamePiece()

        );
    }
}
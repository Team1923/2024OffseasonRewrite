// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.robot.lib.simulation.SimulationSubsystem;
import frc.robot.lib.tuningwidgets.SwerveRequestPIDWidget;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

public class Robot extends TimedRobot {

  private StateHandler stateHandler = StateHandler.getInstance();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    if (Utils.isSimulation()){
      SimulationSubsystem.getInstance();
    }

    SwerveRequestPIDWidget ampPID = new SwerveRequestPIDWidget(SwerveStates.FACING_AMP);
      SwerveRequestPIDWidget GCPID = new SwerveRequestPIDWidget(SwerveStates.GOAL_CENTRIC);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    


    SmartDashboard.putNumber("TX", stateHandler.llTx());

    SmartDashboard.putNumber("TY", stateHandler.llTy());

   
  }

  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

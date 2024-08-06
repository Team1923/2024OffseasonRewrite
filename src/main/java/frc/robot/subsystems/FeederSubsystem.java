// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

  public enum FeederStates {
    OFF(new DutyCycleOut(0)),
    FEED_TO_SHOOTER(new DutyCycleOut(0.85)), // intake -> shooter
    FEED_TO_INTAKE(new DutyCycleOut(-0.8)), // shooter -> intake
    FRONTING(new DutyCycleOut(0.1)),
    BACKING(new DutyCycleOut(-0.1)),
    FULL_EJECT(new DutyCycleOut(0.85));

    public ControlRequest OUTPUT;

    private FeederStates(ControlRequest output) {
      OUTPUT = output;
    }
  }

  private TalonFX feederMotor = new TalonFX(FeederConstants.feederID, "rio");

  /*
   * Beam Break initializations. These are DigitalInput objects that return
   * true/false.
   */
  private DigitalInput beamBreakTwo = new DigitalInput(FeederConstants.beamBreak2ID);
  private DigitalInput beamBreakThree = new DigitalInput(FeederConstants.beamBreak3ID);

  /* CONFIGURATION METHODS */
  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {

    feederMotor.getConfigurator().apply(FeederConstants.CONFIGS);
  }

  /* FUNCTIONAL METHODS */

  public void setFeederTo(ControlRequest output) {
    feederMotor.setControl(output);
  }

  /* INFO METHODS */
  public double getFeederPercent() {
    return feederMotor.get();
  }

  public boolean isAtState(FeederStates state) {
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    StateHandler.getInstance().bb2Covered = !beamBreakTwo.get();
    StateHandler.getInstance().bb3Covered = !beamBreakThree.get();
  }
}

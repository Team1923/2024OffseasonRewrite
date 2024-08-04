// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateHandler;

public class IntakeSubsystem extends SubsystemBase {
  public static enum IntakeStates {
    DEPLOYED(new IntakePosition(2.00)),
    STOWED(new IntakePosition(0));

    private IntakePosition intakePosition;

    private IntakeStates(IntakePosition i) {
        this.intakePosition = i;
    }

    public IntakePosition getIntakePosition() {
        return intakePosition;
    }

    public static class IntakePosition {
      private double angularSetpoint;

      public IntakePosition(double a) {
          this.angularSetpoint = a;
      }

      public double getAngularSetpoint() {
          return angularSetpoint;
      }
  }
}

public static enum IntakeRollerSpeeds {
  OFF(new DutyCycleOut(0)),
  EJECT(new PercentOutputValue(0.75)),
  INTAKE(new PercentOutputValue(-0.85));

  private PercentOutputValue percentOutputValue;

  private IntakeRollerSpeeds(PercentOutputValue p) {
      this.percentOutputValue = p;
  }

  public PercentOutputValue getPercentOutputValue() {
      return percentOutputValue;
  }
}

  /** Creates a new IntakeSubsystem. */
   private DigitalInput beamBreakOne = new DigitalInput(IntakeConstants.beamBreakOneID);
  private StateHandler stateHandler = StateHandler.getInstance();

  private TalonFX intakeArmPrimary = new TalonFX(Constants.IntakeConstants.intakeArmPrimaryID);
  private TalonFX intakeArmFollower = new TalonFX(Constants.IntakeConstants.intakeArmFollowerID);

  private TalonFX intakeWheelTop = new TalonFX(Constants.IntakeConstants.intakeWheelTopID);
  private TalonFX intakeWheelBottom = new TalonFX(Constants.IntakeConstants.intakeWheelBottomID);


  public IntakeSubsystem() {
    intakeArmPrimary.getConfigurator().apply(IntakeConstants.CONFIGS);
    intakeArmFollower.getConfigurator().apply(IntakeConstants.CONFIGS);

    intakeWheelBottom.getConfigurator().apply(IntakeConstants.CONFIGS);
    intakeWheelTop.getConfigurator().apply(IntakeConstants.CONFIGS);

    intakeArmPrimary.setInverted(true);
    intakeArmFollower.setInverted(true);

    // Need to change/test in lab
    intakeArmFollower.setControl(new Follower(IntakeConstants.intakeArmPrimaryID, false));


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

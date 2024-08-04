// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateHandler;

public class IntakeSubsystem extends SubsystemBase {
  public static enum ArmStates {
    DEPLOYED(new MotionMagicVoltage(2.00)),
    STOWED(new MotionMagicVoltage(0));

    public ControlRequest intakePosition;

    private ArmStates(ControlRequest i) {
        this.intakePosition = i;
    }


}

public static enum RollerStates {
  OFF(new DutyCycleOut(0)),
  EJECT(new DutyCycleOut(0.75)),
  INTAKE(new DutyCycleOut(-0.85));

  private ControlRequest OUTPUT;
  private RollerStates(ControlRequest OUTPUT){
    this.OUTPUT = OUTPUT;
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
    intakeArmPrimary.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);
    intakeArmFollower.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);

    intakeWheelBottom.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);
    intakeWheelTop.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);

    intakeArmFollower.setControl(new Follower(IntakeConstants.intakeArmPrimaryID, false));


  }

  // setter method for arm should be in degrees

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

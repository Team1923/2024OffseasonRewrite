// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateHandler;

public class IntakeSubsystem extends SubsystemBase {
  /* Arm States Enum */
  public static enum ArmStates {
    DEPLOYED(MMVoltageWithDegrees(114.592)), // original: 2.00 radians
    STOWED(MMVoltageWithDegrees(0));

    public ControlRequest intakePosition;

    private ArmStates(ControlRequest request) {
      this.intakePosition = request;
    }

    private static MotionMagicVoltage MMVoltageWithDegrees(double degrees) {
      return new MotionMagicVoltage(degrees * IntakeConstants.intakeDegreesToRotations);
    }
  }

  /* Roller States Enum */
  public static enum RollerStates {
    OFF(new DutyCycleOut(0)),
    EJECT(new DutyCycleOut(0.75)),
    INTAKE(new DutyCycleOut(-0.85));

    private ControlRequest OUTPUT;

    private RollerStates(ControlRequest OUTPUT) {
      this.OUTPUT = OUTPUT;
    }
  }

  private DigitalInput beamBreakOne = new DigitalInput(IntakeConstants.beamBreakOneID);

  private TalonFX intakeArmPrimary = new TalonFX(Constants.IntakeConstants.intakeArmPrimaryID);
  private TalonFX intakeArmFollower = new TalonFX(Constants.IntakeConstants.intakeArmFollowerID);

  private TalonFX intakeWheelTop = new TalonFX(Constants.IntakeConstants.intakeWheelTopID);
  private TalonFX intakeWheelBottom = new TalonFX(Constants.IntakeConstants.intakeWheelBottomID);

  /* Constructor */
  public IntakeSubsystem() {
    intakeArmPrimary.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);
    intakeArmFollower.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);

    intakeWheelBottom.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);
    intakeWheelTop.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);

    intakeArmFollower.setControl(new Follower(IntakeConstants.intakeArmPrimaryID, false));
    intakeWheelBottom.setControl(new Follower(IntakeConstants.intakeWheelTopID, false));
  }

  /**
   * Method to get the current intake arm's position (in degrees)
   * @return a double value representing the intake's position in degrees
   */
  public double getIntakeArmPositionDegrees() {
    return intakeArmPrimary.getPosition().getValueAsDouble() * IntakeConstants.intakeRotsToDegrees;
  }

  /**
   * Method to determine if the intake has reached its desired position.
   * @param state the state specifying the desired position
   * @return a boolean representing if the intake arm has reached its desired position
   */
  public boolean isAtArmPosition(ArmStates state) {
      double desiredPosition = ((MotionMagicVoltage) state.intakePosition).Position
          * IntakeConstants.intakeRotsToDegrees;

      return Math.abs(getIntakeArmPositionDegrees() - desiredPosition) < IntakeConstants.intakePositionAllowableOffset;

  }

  /**
   * Method to determine if the intake roller wheels have reached their desired setpoint.
   * @param state the desired RollerState
   * @return true (always)
   */
  public boolean isAtRollerVelocity(RollerStates state) {
    return true;
  }


  /**
   * Method to set the intake arm to a specific ControlRequest
   * @param output the specified desired ControlRequest.
   */
  public void setIntakeArmTo(ControlRequest output) {
    intakeArmPrimary.setControl(output);
  }

  /**
   * Method to set the intake's rollers to a specific ControlRequest.
   * @param output the specified desired ControlRequest.
   */
  public void setRollerSpeedTo(ControlRequest output) {
    intakeWheelTop.setControl(output);
  }

  /**
   * Method to stop the intake arm's motors.
   */
  public void stopIntakeArmMotors() {
    intakeArmPrimary.set(0);
  }

  /**
   * Method ot stop the intake wheels.
   */
  public void stopIntakeWheels() {
    intakeWheelTop.set(0);
    ;
  }

  /**
   * Method to zero the intake arm.
   */
  public void zeroIntakeArm() {
    intakeArmPrimary.setPosition(0);
    intakeArmFollower.setPosition(0);
  }

  @Override
  public void periodic() {
    StateHandler.getInstance().bb1Covered = !beamBreakOne.get();
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  public static enum ArmStates {
    STOWED(MMVoltageWithDegrees(0)),
    REVERSE_SUBWOOFER(MMVoltageWithDegrees(-112.3)),
    UNGUARDABLE(MMVoltageWithDegrees(-112.3)),
    AMP(MMVoltageWithDegrees(-112.3)),
    SPEAKER(MMVoltageWithDegrees(-44.1)),
    TRAP(MMVoltageWithDegrees(-51.6)),
    BABY_BIRD(MMVoltageWithDegrees(-40.1)),
    PUNT_HIGH(MMVoltageWithDegrees(-38.4)),
    PUNT_LOW(MMVoltageWithDegrees(0)),
    FRONT_AMP(MMVoltageWithDegrees(-44.1)),
    DEFENSE(MMVoltageWithDegrees(-77.3)),
    CLIMB(MMVoltageWithDegrees(-77.3));

    public ControlRequest armOutput;

    private ArmStates(ControlRequest OUTPUT) {
      this.armOutput = OUTPUT;
    }

    private static MotionMagicVoltage MMVoltageWithDegrees(double degrees) {
      return new MotionMagicVoltage(degrees * ArmConstants.armDegreesToRots);
    }
  }

  /* Initialize arm motors */
  private TalonFX armPrimary = new TalonFX(ArmConstants.armMotorPrimaryID, "rio");
  private TalonFX armFollower = new TalonFX(ArmConstants.armMotorFollowerID, "rio");

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armPrimary.getConfigurator().apply(ArmConstants.CONFIGS);
    armFollower.getConfigurator().apply(ArmConstants.CONFIGS);

    armFollower.setControl(new Follower(ArmConstants.armMotorPrimaryID, true));

    zeroArm();
  }

  /**
   * Method to set the arm position.
   * 
   * @param output the specified ControlRequest to output the motor at.
   */
  public void setArmPosition(ControlRequest output) {
    armPrimary.setControl(output);
  }

  /**
   * Move the arm using percent output. Primarily used for testing purposes.
   * 
   * @param out percent out speed to run the arm at
   */
  public void setPercentOut(double out) {
    armPrimary.set(out);
  }

  /**
   * Get the position of the arm, converted from the encoder reading.
   * 
   * @return The arm position in degrees.
   */
  public double getArmPositionDegrees() {
    return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToDegrees;
  }

  /**
   * Gets the position of the arm directly from the encoder reading.
   * 
   * @return The arm position in rotations.
   */
  public double getArmPositionRots() {
    return armPrimary.getPosition().getValueAsDouble();
  }

  /**
   * Method to determine if the intake has reached its desired position.
   * 
   * @param state the state specifying the desired position
   * @return a boolean representing if the intake arm has reached its desired
   *         position
   */
  public boolean isAtArmPosition(ArmStates state) {
    double desiredPosition = ((MotionMagicVoltage) state.armOutput).Position
        * ArmConstants.armRotsToDegrees;

    return Math.abs(getArmPositionDegrees() - desiredPosition) < ArmConstants.armPositionAllowableOffset;

  }

  /**
   * Method to stop the arm's motors.
   */
  public void stopArmMotors() {
    armPrimary.stopMotor();
  }

  /**
   * Method to zero the arm.
   */
  public void zeroArm() {
    armPrimary.setPosition(0);
    armFollower.setPosition(0);
  }

  @Override
  public void periodic() {

  }
}

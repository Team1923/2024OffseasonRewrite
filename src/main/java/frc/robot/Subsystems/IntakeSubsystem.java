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


  /* Constructor */
  public IntakeSubsystem() {
    intakeArmPrimary.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);
    intakeArmFollower.getConfigurator().apply(IntakeConstants.ARM_CONFIGS);

    intakeWheelBottom.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);
    intakeWheelTop.getConfigurator().apply(IntakeConstants.WHEEL_CONFIGS);

    intakeArmFollower.setControl(new Follower(IntakeConstants.intakeArmPrimaryID, false));
    intakeWheelBottom.setControl(new Follower(IntakeConstants.intakeWheelTopID, false));

  }

  /*Getter Methods*/
  public double getWheelTopRPS(){
    return intakeWheelTop.getVelocity().getValueAsDouble();
  }
   public double getWheelBottomRPS(){
    return intakeWheelBottom.getVelocity().getValueAsDouble();
  }

  // Change to degrees
  public double getIntakeArmPositionDegrees() {
    return intakeArmPrimary.getPosition().getValueAsDouble() * IntakeConstants.intakeRotsToDegrees;
  }

  /* isAt boolean checker methods */
  public boolean isAtArmPosition(ArmStates state){
    if (state.intakePosition instanceof MotionMagicVoltage){
      // Is this the correct way to get the desired position
      double desiredPosition = ((MotionMagicVoltage)state.intakePosition).Position * IntakeConstants.intakeRotsToDegrees;
      
      return Math.abs(getIntakeArmPositionDegrees() - desiredPosition) < IntakeConstants.intakePositionAllowableOffset;
    }
    else{ //Timer for percent out?
      return true;
    }
  }

  public boolean isAtRollerVelocity(RollerStates state) {
    return true;
  }

  // setter method for arm should be in degrees

  /* setter methods */
  public void setIntakeArmTo(ControlRequest Output) {
    intakeArmPrimary.setControl(Output);
  }

  public void setRollerSpeedTo(ControlRequest Output) {
    intakeWheelTop.setControl(Output);
  }

  /* stop/zero methods */
  public void stopIntakeArmMotors() {
    intakeArmPrimary.stopMotor();;
  }

  public void stopIntakeWheels() {
    intakeWheelTop.stopMotor();;
  }

  /* TODO: Check this */
  public void zeroIntakeArm() {
    intakeArmPrimary.setPosition(0);
    intakeArmFollower.setPosition(0);
  }


  @Override
  public void periodic() {

    StateHandler.getInstance().bb1Covered = !beamBreakOne.get();
    // This method will be called once per scheduler run
  }
}

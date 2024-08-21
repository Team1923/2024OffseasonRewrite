// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.lib.tuningwidgets.MotorPIDFVAJWidget;

public class ArmSubsystem extends SubsystemBase {

  public static enum ArmStates {
    STOWED(MMVoltageWithDegrees(0)),
    REVERSE_SUBWOOFER(MMVoltageWithDegrees(-108.5)),
    UNGUARDABLE(MMVoltageWithDegrees(-108.5)),
    // AMP(MMVoltageWithDegrees(-108.5)),
    SUBWOOFER(MMVoltageWithDegrees(-44.1)), 
    RANGED(MMVoltageWithDegrees(0).withSlot(1)),
    TRAP(MMVoltageWithDegrees(-51.6), 0.5),
    BABY_BIRD(MMVoltageWithDegrees(-40.1)), 
    PUNT_HIGH(MMVoltageWithDegrees(-38.4)),
    PUNT_LOW(MMVoltageWithDegrees(0)),
    FRONT_AMP(MMVoltageWithDegrees(-44.1)),
    DEFENSE(MMVoltageWithDegrees(-77.3)),
    CLIMB(MMVoltageWithDegrees(-77.3)),
    ZEROING(new DutyCycleOut(0.05)),
    ANGLE_TUNING(MMVoltageWithDegrees(0).withSlot(1)),
    OFF(new DutyCycleOut(0));

    public ControlRequest REQUEST;
    public double settleTime;

    private ArmStates(ControlRequest request) {
      this.REQUEST = request;
      this.settleTime = 0;
    }

    private ArmStates(ControlRequest request, double settleTime) {
      this.REQUEST = request;
      this.settleTime = settleTime;
    }

    private static MotionMagicVoltage MMVoltageWithDegrees(double degrees) {
      // return new MotionMagicVoltage(degrees * ArmConstants.armDegreesToRots);
      return new MotionMagicVoltage(Units.degreesToRotations(degrees));

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

    // MotorPIDFVAJWidget armTuning = new MotorPIDFVAJWidget("ARM", ArmConstants.CONFIGS, 1, ArmConstants.armRotsToDegrees, 0, armPrimary, armFollower);
    MotorPIDFVAJWidget armTuning = new MotorPIDFVAJWidget("ARM", ArmConstants.CONFIGS, 1, 360, 0, armPrimary, armFollower);

    zeroArm();
  }

  /**
   * Method to set the arm position.
   * 
   * @param output the specified ControlRequest to output the motor at.
   */
  public void setArmTo(ControlRequest output) {
    armPrimary.setControl(output);
    // System.out.println("request :" + output.getControlInfo().toString());
  }

  /**
   * Move the arm using percent output. Primarily used for testing purposes.
   * 
   * @param out percent out speed to run the arm at
   */
  public void setPercentOut(double out) {
    armPrimary.setControl(new DutyCycleOut(out));
  }

  /**
   * Get the position of the arm, converted from the encoder reading.
   * 
   * @return The arm position in degrees.
   */
  public double getArmPositionDegrees() {
    // return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToDegrees;
    return Units.rotationsToDegrees(armPrimary.getPosition().getValueAsDouble());

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
   * Gets the supply current of the arm primary motor
   * @return arm supply current in Amps
   */
  public double getArmSupplyCurrent(){
    return armPrimary.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Method to determine if the intake has reached its desired position.
   * 
   * @param state the state specifying the desired position
   * @return a boolean representing if the intake arm has reached its desired
   *         position
   */
  public boolean isAtState(ArmStates state) {

    if (Utils.isSimulation()) return true;

    if (state.REQUEST instanceof MotionMagicVoltage){
      // double desiredPosition = ((MotionMagicVoltage) state.REQUEST).Position * ArmConstants.armRotsToDegrees;
      double desiredPositionDegrees = Units.rotationsToDegrees(((MotionMagicVoltage) state.REQUEST).Position);


      return Math.abs(getArmPositionDegrees() - desiredPositionDegrees) < ArmConstants.armPositionAllowableOffset;
    }
    else{
      return true;
    }
    

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

    // SmartDashboard.putString("PRIMARY MODE", armPrimary.getAppliedControl().getName());
    //     SmartDashboard.putString("FOLLOWER MODE", armFollower.getAppliedControl().getName());

    SmartDashboard.putNumber("Primary arm degrees", getArmPositionDegrees());
    SmartDashboard.putNumber("Primary arm rots", getArmPositionRots());
    // SmartDashboard.putNumber("Follower arm", armFollower.getPosition().getValueAsDouble() * ArmConstants.armRotsToDegrees);

    SmartDashboard.putNumber("Arm Supply", getArmSupplyCurrent());

  }
}

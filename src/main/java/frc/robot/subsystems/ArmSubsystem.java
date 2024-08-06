// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public static enum States {
        STOWED(new ArmPosition(0)),
        REVERSE_SUBWOOFER(new ArmPosition(-1.96)),
        UNGUARDABLE(new ArmPosition(-1.96)),
        AMP(new ArmPosition(-1.96)), 
        SPEAKER(new ArmPosition(-0.77)), //THIS IS A DEFAULT VALUE FOR SUBWOOFER SHOOTING - (-0.77)
        TRAP(new ArmPosition(-0.9)), //TODO: FIND
        BABY_BIRD(new ArmPosition(-0.7)), //TODO: FIND
        PUNT_HIGH(new ArmPosition(-0.67)),
        PUNT_LOW(new ArmPosition(0)),
        FRONT_AMP(new ArmPosition(-0.77)),
        DEFENSE(new ArmPosition(-1.35)),
        CLIMB(new ArmPosition(-1.35));


        public ControlRequest primary;
        public ControlRequest follower;

        private States(ControlRequest OUTPUT) {
            this.primary = OUTPUT;
            this.follower = OUTPUT;
        }

        private States(ControlRequest primary, ControlRequest follower) {
            this.primary = primary;
            this.follower = follower;
        }
    }

    private TalonFX armPrimary = new TalonFX(ArmConstants.armMotorPrimaryID, "rio");
    private TalonFX armFollower = new TalonFX(ArmConstants.armMotorFollowerID, "rio");

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armPrimary.getConfigurator().apply(ShooterConstants.CONFIGS);
    armFollower.getConfigurator().apply(ShooterConstants.CONFIGS);

    armFollower.setControl(new Follower(ArmConstants.armMotorPrimaryID, true));
  }

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
       * Get the position of the arm from the encoder reading.
       * 
       * @return The arm position in radians.
       */
      public double getArmPositionRads() {
        return armPrimary.getPosition().getValueAsDouble() * ArmConstants.armRotsToRads;
      }
    
      /**
       * Gets the position of the arm, converted from the encoder reading.
       * 
       * @return The arm position in rotations.
       */
      public double getArmPositionRots() {
        return armPrimary.getPosition().getValueAsDouble();
      }

      public boolean isAtArmState(double desiredSetpoint) {
        return Math.abs(getArmPositionRads() - desiredSetpoint) < ArmConstants.armPositionAllowableOffset; 
      }


    @Override
    public void periodic() {
        
    }
}

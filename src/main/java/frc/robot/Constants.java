package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {

        public static class ControllerConstants {
                public static class Driver {
                        public static final double deadband = 0.07;
                }
        }

        public static class ShooterConstants {
                /* Shooter Motor IDs */
                public static final int shooterTopID = 17;
                public static final int shooterBottomID = 18;

                /* Blower Motor ID */
                public static final int blowerID = 30;

                /* BB4 ID */
                public static final int beamBreak4ID = 4;

                /* Conversion Factors */
                public static final double RPSToRPM = 60;
                public static final double RPMToRPS = 1 / RPSToRPM;

                /* Parameters for the Shooter's Acceleration and Jerk */
                public static final double maxShooterAccel = 400; // Rotations/sec^2
                public static final double maxShooterJerk = 5000; // Rotations/sec^3

                /* TalonFX Motor Configuration for the Shooter */
                public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()
                                .withSlot0(new Slot0Configs() // PID
                                                .withKP(0.2)
                                                .withKI(0)
                                                .withKD(0)
                                                .withKS(0.25)
                                                .withKA(0)
                                                .withKV(0.115))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicAcceleration(maxShooterAccel)
                                                .withMotionMagicJerk(maxShooterJerk))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true));

                /* RPM Threshold for Current State Evaluation */
                public static final double shooterRPMThreshhold = 25;

        }

        public static class IntakeConstants {
                /* Intake Arm Motor IDs */
                public static int intakeArmPrimaryID = 22;
                public static int intakeArmFollowerID = 21;

                /* Intake Wheels Motor IDs */
                public static int intakeWheelTopID = 25;
                public static int intakeWheelBottomID = 26;

                /* BB1 ID (port on RoboRIO) */
                public static int beamBreakOneID = 1;

                /* Parameters for the intake's motion */
                public static final double maxIntakeVel = 100;
                public static final double maxIntakeAccel = 500;
                public static final double maxIntakeJerk = 2200;

                /* Gearbox ratios and unit conversions */
                public static final double intakeGearRatio = 60;
                public static final double intakeRotsToDegrees = 360 / intakeGearRatio;
                public static final double intakeDegreesToRotations = 1 / intakeRotsToDegrees;

                /* Define allowable offset for the intake arm's position */
                public static final double intakePositionAllowableOffset = 2.864789;

                /* Arm Configuration - NOTE: clockwise positive is correct */
                public static final TalonFXConfiguration ARM_CONFIGS = new TalonFXConfiguration()
                                .withSlot0(new Slot0Configs() // PID
                                                .withKP(0.9)
                                                .withKI(0.005)
                                                .withKD(0)
                                                .withKS(0)
                                                .withKV(0.1))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(maxIntakeVel)
                                                .withMotionMagicAcceleration(maxIntakeAccel)
                                                .withMotionMagicJerk(maxIntakeJerk))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true));

                public static final TalonFXConfiguration WHEEL_CONFIGS = new TalonFXConfiguration();
        }

        public static class FeederConstants {
                public static final int feederID = 14;

                public static final int beamBreak2ID = 2;
                public static final int beamBreak3ID = 3;

                public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true));
        }

}

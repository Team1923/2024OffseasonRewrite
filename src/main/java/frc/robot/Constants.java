package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Constants {

    public static class ControllerConstants{
        public static class Driver{
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

        /* Parameter's for the Shooter's Acceleration and Jerk */
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


}

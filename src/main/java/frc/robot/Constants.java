package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class Constants {

    public static class ShooterConstants {

        public static final int shooterTopID = 17;
        public static final int shooterBottomID = 18;

        public static final int blowerID = 30;

        public static final int beamBreak4ID = 4;

        public static final double RPSToRPM = 60;
        public static final double RPMToRPS = 1 / RPSToRPM;

        public static final double maxShooterAccel = 400; // Rotations/sec^2
        public static final double maxShooterJerk = 5000; // Rotations/sec^3

        // NOTE: Decided to ignore motionmagicvelocity control
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
                .withCurrentLimits(new CurrentLimitsConfigs() // TODO: Take a look at this (no refresh?)
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));

        public static final double shooterRPMThreshhold = 50; // RPM

    }

    public static class ArmConstants {

        public static final int armMotorPrimaryID = 15; // right
        public static final int armMotorFollowerID = 16; // left

        /* Motion Magic Constants */
        public static final double armKS = 0;
        public static final double armkP = 1.2;
        public static final double armkI = 0.005;
        public static final double armkD = 0;
        public static final double maxArmVel = 100;
        public static final double maxArmAccel = 250;
        public static final double maxArmJerk = 1000;

        /* Gearbox Ratios & Unit Conversions */
        public static final double armGearRatio = 129.6;
        public static final double armRotsToRads = (2 * Math.PI) / armGearRatio;
        public static final double armRadsToRots = armGearRatio / (2 * Math.PI);

        /* kG - gravity constant for motion of arm */
        public static final double armMaxGravityConstant = 0.03 * 12; // 2 volts max ff

        public static final double armPositionAllowableOffset = 0.03; // allowed radians offset

        public static final double armPositionChange = 0.01; // TODO: Tune at Lehigh!!!!

        public static final double armSettleTime = 0.5;

        public static final double ArmRPMThreshhold = 50; // RPM



        // NOTE: Decided to ignore motionmagicvelocity control
        public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()
                .withSlot0(new Slot0Configs() // PID
                        .withKP(armkP)
                        .withKI(armkI)
                        .withKD(armkD)
                        .withKS(armKS)
                        .withKA(0) // NOT SURE WHAT TO PUT HERE
                        .withKV(0.115) // NOT SURE WHAT TO PUT HERE
                )
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(maxArmAccel)
                        .withMotionMagicJerk(maxArmJerk))
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs() // TODO: Take a look at this (no refresh?)
                        .withStatorCurrentLimit(80)
                        .withStatorCurrentLimitEnable(true));

            

    }

}

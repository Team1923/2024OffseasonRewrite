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


    public static class ShooterConstants{

        public static final int shooterTopID = 17;
        public static final int shooterBottomID = 18;

        public static final int blowerID = 30;

        public static final int beamBreak4ID = 4;
        
        public static final double RPSToRPM = 60;
        public static final double RPMToRPS = 1/RPSToRPM;

        public static final double maxShooterAccel = 400; //Rotations/sec^2
        public static final double maxShooterJerk = 5000; //Rotations/sec^3

        

        //NOTE: Decided to ignore motionmagicvelocity control
        public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()
                                                            .withSlot0(new Slot0Configs() //PID
                                                                            .withKP(0.2)
                                                                            .withKI(0)
                                                                            .withKD(0)
                                                                            .withKS(0.25)
                                                                            .withKA(0)
                                                                            .withKV(0.115)
                                                                        )
                                                            .withMotionMagic(new MotionMagicConfigs()
                                                                                .withMotionMagicAcceleration(maxShooterAccel)
                                                                                .withMotionMagicJerk(maxShooterJerk))
                                                            .withMotorOutput(new MotorOutputConfigs()
                                                                            .withNeutralMode(NeutralModeValue.Brake)
                                                                            )
                                                            .withCurrentLimits(new CurrentLimitsConfigs() //TODO: Take a look at this (no refresh?)
                                                                                .withStatorCurrentLimit(80)
                                                                                .withStatorCurrentLimitEnable(true)
                                                                            );


        

        public static final double shooterRPMThreshhold = 50; //RPM
                                                        
    }



}

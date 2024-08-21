package frc.robot;

import org.opencv.core.Point;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Constants {

        public static class ControllerConstants {
                public static class Driver {
                        public static final double deadband = 0.07;
                }

                public static class Operator {
                        public static final double deadband = 0.1;
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
                public static int beamBreak1ID = 1;

                /* Parameters for the intake's motion */
                public static final double maxIntakeVel = 100;
                public static final double maxIntakeAccel = 500;
                public static final double maxIntakeJerk = 2200;

                /* Gearbox ratios and unit conversions */
                public static final double intakeGearRatio = 60;
                public static final double intakeRotsToDegrees = 360 / intakeGearRatio;
                public static final double intakeDegreesToRotations = 1 / intakeRotsToDegrees;

                /* Define allowable offset for the intake arm's position */
                public static final double intakePositionAllowableOffset = 2.9;

                /* Arm Configuration - NOTE: clockwise positive is correct */
                public static final TalonFXConfiguration ARM_CONFIGS = new TalonFXConfiguration()
                                .withSlot0(new Slot0Configs() // PID
                                                .withKP(0.9)
                                                .withKI(0.005)
                                                .withKD(0)
                                                .withKS(0)
                                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                                                .withKV(0.1))
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(maxIntakeVel)
                                                .withMotionMagicAcceleration(maxIntakeAccel)
                                                .withMotionMagicJerk(maxIntakeJerk))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Coast) 
                                                .withInverted(InvertedValue.Clockwise_Positive))
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true))
                                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1/intakeGearRatio));


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
                
                public static final double timeout = 2; //seconds
        }

        public static class ArmConstants {
                /* Motor IDs */
                public static final int armMotorPrimaryID = 15; // right
                public static final int armMotorFollowerID = 16; // left

                /* Motion Magic Constants */
                // public static final double armKS = 0;
                // public static final double armkP = 1.2;
                // public static final double armkI = 0.005;
                // public static final double armkD = 0;

                //Faster motion magic 
                // public static final double maxArmVel = 300;
                // public static final double maxArmAccel = 175;
                // public static final double maxArmJerk = 1000;

                //Slower motion magic
                public static final double maxArmVel = 190;
                public static final double maxArmAccel = 60;
                public static final double maxArmJerk = 500;

                /* Gearbox Ratios & Unit Conversions */
                public static final double armGearRatio = 129.6;
                public static final double armRotsToDegrees = 360 / armGearRatio;
                public static final double armDegreesToRots = 1 / armRotsToDegrees;

                /* kG - gravity constant for motion of arm */
                public static final double armMaxGravityConstant = 0.05 * 12; // 2 volts max ff

                public static final double armPositionAllowableOffset = 1;

                public static final TalonFXConfiguration CONFIGS = new TalonFXConfiguration()

                                .withSlot0(new Slot0Configs() // Fast PID
                                                .withKP(2)
                                                .withKI(0)
                                                .withKD(0.01)
                                                .withKS(0.05)
                                                .withKG(0.04*12)
                                                .withGravityType(GravityTypeValue.Arm_Cosine))        
                                .withSlot1(new Slot1Configs() //Accurate PID
                                                .withKP(2)
                                                .withKI(0.1)
                                                .withKD(0)
                                                .withKS(0.09)
                                                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                                                .withKG(armMaxGravityConstant)
                                                .withGravityType(GravityTypeValue.Arm_Cosine))       
                                .withMotionMagic(new MotionMagicConfigs()
                                                .withMotionMagicCruiseVelocity(maxArmVel)
                                                .withMotionMagicAcceleration(maxArmAccel)
                                                .withMotionMagicJerk(maxArmJerk))
                                .withMotorOutput(new MotorOutputConfigs()
                                                .withNeutralMode(NeutralModeValue.Brake)) 
                                .withCurrentLimits(new CurrentLimitsConfigs()
                                                .withStatorCurrentLimit(80)
                                                .withStatorCurrentLimitEnable(true))
                                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1/armGearRatio));
                
                // public static final double armSupplyToZero = 0.5; //Amps

                // public static final double armOffsetAtHardstop = 0; //the amount of degrees the arm is at the hardstop BELOW our normal zero

        }



        public static final class FieldConstants{
                public static final Pose2d blueSpeakerPos = new Pose2d(-0.038099999999999995, 5.547867999999999, Rotation2d.fromDegrees(180.0));
                public static final Pose2d redSpeakerPos = new Pose2d(16.579342, 5.547867999999999, Rotation2d.fromDegrees(0));
                
                public static final Point blueSourceStart = new Point(14, 0);
                public static final Point blueSourceEnd = new Point(16.6, 1.7);
                
                
                
                public static final Point redSourceStart = new Point(0.135, 1.562);
                public static final Point redSourceEnd = new Point(1.732, 0.392);
        }

        public static final class LEDConstants{
                public static final int LEDCount = 41 + 8;
                public static final int CANdleID = 23;
        
        
        }

        public static final class LimelightConstants{
                public static final String limelightName = "limelight-shooter";

                public static final double centeredTolerance = 2;
        }

        public static final class InterpolationConstants {


                public static InterpolatingDoubleTreeMap tyToDistanceMap = new InterpolatingDoubleTreeMap();

                public static InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
                public static InterpolatingDoubleTreeMap distanceToAngle = new InterpolatingDoubleTreeMap();
                
                public InterpolationConstants(){
                        fillTyDistMap();
                        fillDistPosRPMMap();
                }
                
                public static void fillTyDistMap(){
                        tyToDistanceMap.put(-14.97, 12.0);
                        tyToDistanceMap.put(-11.99, 18.0);
                        tyToDistanceMap.put(-9.36, 24.0);
                        tyToDistanceMap.put(-7.03, 30.0);
                        tyToDistanceMap.put(-4.64, 36.0);
                        tyToDistanceMap.put(-2.72, 42.0);
                        tyToDistanceMap.put(-0.80, 48.0);
                        tyToDistanceMap.put(0.59, 54.0);
                        tyToDistanceMap.put(2.24, 60.0);
                        tyToDistanceMap.put(3.74, 66.0);
                        tyToDistanceMap.put(5.40, 72.0);
                        tyToDistanceMap.put(6.53, 78.0);
                        tyToDistanceMap.put(7.26, 84.0);
                        tyToDistanceMap.put(7.93, 90.0);

                }

                public static void fillDistPosRPMMap(){
                        distanceToAngle.put(12.0, -44.0); distanceToRPM.put(12.0, 2000.0);
                        distanceToAngle.put(18.0, -41.0); distanceToRPM.put(18.0, 2100.0);
                        distanceToAngle.put(24.0, -40.0); distanceToRPM.put(24.0, 2200.0);
                        distanceToAngle.put(30.0, -39.0); distanceToRPM.put(30.0, 2200.0);
                        distanceToAngle.put(36.0, -38.0); distanceToRPM.put(36.0, 2300.0);
                        distanceToAngle.put(42.0, -37.0); distanceToRPM.put(42.0, 2300.0);
                        distanceToAngle.put(48.0, -35.0); distanceToRPM.put(48.0, 2400.0);
                        distanceToAngle.put(54.0, -32.0); distanceToRPM.put(54.0, 2500.0);
                        distanceToAngle.put(60.0, -31.0); distanceToRPM.put(60.0, 2600.0);
                        distanceToAngle.put(66.0, -29.0); distanceToRPM.put(66.0, 2700.0);
                        distanceToAngle.put(72.0, -27.0); distanceToRPM.put(72.0, 2800.0);
                        distanceToAngle.put(78.0, -25.0); distanceToRPM.put(78.0, 2900.0);
                        distanceToAngle.put(84.0, -24.0); distanceToRPM.put(84.0, 3000.0);
                        distanceToAngle.put(90.0, -23.0); distanceToRPM.put(90.0, 3200.0);
                        distanceToAngle.put(96.0, -19.0); distanceToRPM.put(96.0, 3300.0);
                        
                        
                }

        }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  public static enum States{ //TODO: fix unit conversions
      IDLE(new MotionMagicVelocityVoltage(0)), 
      BABY_BIRD_INTAKE(new MotionMagicVelocityVoltage(-1000 * ShooterConstants.RPMToRPS)),
      FRONT_AMP_SHOT(new MotionMagicVelocityVoltage(415 * ShooterConstants.RPMToRPS)),
      UNGUARDABLE_SHOT(new MotionMagicVelocityVoltage(1190 * ShooterConstants.RPMToRPS), new MotionMagicVelocityVoltage(1905 * ShooterConstants.RPMToRPS)),
      TRAP_SHOT(new MotionMagicVelocityVoltage(650 * ShooterConstants.RPMToRPS), new MotionMagicVelocityVoltage(1000 * ShooterConstants.RPMToRPS)),
      PUNT_HIGH_SHOT(new MotionMagicVelocityVoltage(2100 * ShooterConstants.RPMToRPS)), 
      PUNT_LOW_SHOT(new DutyCycleOut(1)), 
      SUBWOOFER_SHOT(new MotionMagicVelocityVoltage(2000 * ShooterConstants.RPMToRPS)),
      REVERSE_SUBWOOFER_SHOT(new MotionMagicVelocityVoltage(2000 * ShooterConstants.RPMToRPS)),
      RANGED_SHOT(new MotionMagicVelocityVoltage(0)),
      FULL_EJECT(new DutyCycleOut(1));

      public ControlRequest OUTPUT_TOP;
      public ControlRequest OUTPUT_BOTTOM;

      private States(ControlRequest OUTPUT){
        this.OUTPUT_TOP = OUTPUT;
        this.OUTPUT_BOTTOM = OUTPUT;
      }

      private States(ControlRequest OUTPUT_TOP, ControlRequest OUTPUT_BOTTOM){
        this.OUTPUT_TOP = OUTPUT_TOP;
        this.OUTPUT_BOTTOM = OUTPUT_BOTTOM;
      }
  }



  private TalonFX shooterTop = new TalonFX(ShooterConstants.shooterTopID);
  private TalonFX shooterBottom = new TalonFX(ShooterConstants.shooterBottomID);

  private TalonSRX blower = new TalonSRX(ShooterConstants.blowerID);

  private DigitalInput beamBreak4 = new DigitalInput(ShooterConstants.beamBreak4ID);
    
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
      shooterTop.getConfigurator().apply(ShooterConstants.CONFIGS);
      shooterBottom.getConfigurator().apply(ShooterConstants.CONFIGS);

      

      blower.configFactoryDefault();

      //No shooter bottom followership

  }


  public void setShooterMotorsTo(ControlRequest output){
    shooterTop.setControl(output);
    shooterBottom.setControl(output);
  }

  public void setShooterMotorsTo(ControlRequest topOutput, ControlRequest bottomOutput){
    shooterTop.setControl(topOutput);
    shooterBottom.setControl(bottomOutput);

  }

  public void setBlowerTo(double percent){
    blower.set(ControlMode.PercentOutput, percent);
  }


  public double getTopRPM(){
    return shooterTop.getVelocity().getValueAsDouble() * ShooterConstants.RPSToRPM;
  }
   public double getBottomRPM(){
    return shooterBottom.getVelocity().getValueAsDouble() * ShooterConstants.RPSToRPM;
  }

  public boolean isAtState(States state){

    if (state.OUTPUT_TOP instanceof MotionMagicVelocityDutyCycle){


      double desiredVelocityTop = ((MotionMagicVelocityDutyCycle)state.OUTPUT_TOP).Velocity;
      double desiredVelocityBottom = ((MotionMagicVelocityVoltage)state.OUTPUT_BOTTOM).Velocity;


      return Math.abs(getTopRPM() - desiredVelocityTop) < ShooterConstants.shooterRPMThreshhold
      && Math.abs(getBottomRPM() - desiredVelocityBottom) < ShooterConstants.shooterRPMThreshhold;
    }
    else{ //Timer for percent out?
      return true;
    }
  }

  // }
  // public boolean isAtSpeed(ControlRequest output){
    
  //   if (output instanceof MotionMagicVelocityDutyCycle){

  //     double desiredVelocity = ((MotionMagicVelocityDutyCycle)output).Velocity;

  //     return Math.abs(getTopRPM() - desiredVelocity) < ShooterConstants.shooterRPMThreshhold
  //     && Math.abs((getBottomRPM() - desiredVelocity)) < ShooterConstants.shooterRPMThreshhold;

  //   }
  //   else{ //Timer for percent out?
  //     return true;
  //   }

  // }

  // public boolean isAtSpeed(ControlRequest outputTop, ControlRequest outputBottom){
    
  //   if (outputTop instanceof MotionMagicVelocityDutyCycle){

  //     double desiredVelocityTop = ((MotionMagicVelocityDutyCycle)outputTop).Velocity;
  //     double desiredVelocityBottom = ((MotionMagicVelocityVoltage)outputBottom).Velocity;

  //     return Math.abs(getTopRPM() - desiredVelocityTop) < ShooterConstants.shooterRPMThreshhold
  //     && Math.abs(getBottomRPM() - desiredVelocityBottom) < ShooterConstants.shooterRPMThreshhold;

  //   }
  //   else{ //Timer for percent out?
  //     return true;
  //   }

  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    StateHandler.getInstance().bb4Covered = !beamBreak4.get();
    

    //TODO: ((MotionMagicVelocityVoltage)(States.RANGED.OUTPUT)).Velocity = updated value;
  }
}

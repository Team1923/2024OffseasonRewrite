// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statecommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.Constants.ControllerConstants;
import frc.robot.lib.swerve.TunerConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.States;

public class SwerveStateMachine extends Command {

  private SwerveSubsystem swerve;


  private StateHandler stateHandler = StateHandler.getInstance();


  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;

  /** Creates a new ManageRequests. */
  /**
   * 
   * @param swerve
   * @param translation Expected to be + away from driver station, - towards driver station
   * @param strafe Expected to be + rightward relative to driver station, - leftward relative to driver station
   * @param rotation Expected to be CCW +
   */
  public SwerveStateMachine(SwerveSubsystem swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.


    this.swerve = swerve;
    this.translationSupplier = translation;
    this.strafeSupplier = strafe;
    this.rotationSupplier = rotation;

    addRequirements(this.swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveSubsystem.States currentState = stateHandler.currentSwerveState;


    double translation = MathUtil.applyDeadband(translationSupplier.getAsDouble() * sideInversions()[0], ControllerConstants.Driver.deadband);
    double strafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble() * sideInversions()[1], ControllerConstants.Driver.deadband);
    double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble() * sideInversions()[2], ControllerConstants.Driver.deadband);

    // if (currentRequest == SwerveRequests.AUTO && !DriverStation.isAutonomous()){
    //   System.out.println("SWERVE STILL THINKS IN AUTO");
    //   currentRequest = SwerveRequests.FIELD_CENTRIC;
    // }

    SwerveRequest request;

    switch(currentState){
      case ROBOT_CENTRIC:
        request = ((SwerveRequest.RobotCentric)SwerveSubsystem.States.ROBOT_CENTRIC.REQUEST)
        .withVelocityX(translation * TunerConstants.kSpeedAt12VoltsMps)
        .withVelocityY(strafe * TunerConstants.kSpeedAt12VoltsMps)
        .withRotationalRate(rotation * Units.degreesToRadians(TunerConstants.kMaxAngularVelocity));
        break;
      case FACING_AMP:
        request = ((SwerveRequest.FieldCentricFacingAngle)SwerveSubsystem.States.FACING_AMP.REQUEST)
        .withVelocityX(translation * TunerConstants.kSpeedAt12VoltsMps)
        .withVelocityY(strafe * TunerConstants.kSpeedAt12VoltsMps)
        .withTargetDirection(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90));
        break;
      case GOAL_CENTRIC:
        // if (hasTag && Math.abs(rotation) < 0.5){
        //   request = ((SwerveRequest.FieldCentricFacingAngle)SwerveSubsystem.States.GOAL_CENTRIC.request)
        //   .withVelocityX(translation * TunerConstants.kSpeedAt12VoltsMps)
        //   .withVelocityY(strafe * TunerConstants.kSpeedAt12VoltsMps)
        //   .withTargetDirection(Rotation2d.fromDegrees(swerve.getGyroYaw().getDegrees()-stateHandler.getxAngleOffset()));
        //   break;
        // }
      case FIELD_CENTRIC: //fall through to default lol
      default:
        request = ((SwerveRequest.FieldCentric)SwerveSubsystem.States.FIELD_CENTRIC.REQUEST)
                  .withVelocityX(translation * TunerConstants.kSpeedAt12VoltsMps)
                  .withVelocityY(strafe * TunerConstants.kSpeedAt12VoltsMps)
                  .withRotationalRate(rotation * Units.degreesToRadians(TunerConstants.kMaxAngularVelocity));

    }
    
    // else if (currentRequest == SwerveRequests.NOTE_SEARCHING){
    //    requestObj =((SwerveRequest.FieldCentricFacingAngle)SwerveRequests.NOTE_SEARCHING.request)
    //     .withVelocityX(0.5 * sideInversions()[0] * TunerConstants.kSpeedAt12VoltsMps)
    //     .withVelocityY(0.5 * sideInversions()[1] * TunerConstants.kSpeedAt12VoltsMps)
    //     .withTargetDirection(Rotation2d.fromDegrees((DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue) ? -90 : -90));

    //     System.out.println("HERER TOO");
    // }

    

    swerve.setControl(request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public static int[] sideInversions(){
    return (DriverStation.getAlliance().get() == Alliance.Blue) ? new int[]{1,1,1}
                                                                : new int[]{-1,-1, 1};
  }
}
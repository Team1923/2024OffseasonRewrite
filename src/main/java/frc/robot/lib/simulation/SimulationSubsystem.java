// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.simulation;

import java.lang.reflect.Field;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.TreeSet;

import javax.xml.transform.Source;

import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.StateHandler;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import frc.robot.subsystems.FeederSubsystem.FeederStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmStates;
import frc.robot.subsystems.IntakeSubsystem.IntakeRollerStates;
import frc.robot.subsystems.ShooterSubsystem.ShooterStates;

public class SimulationSubsystem extends SubsystemBase {

  private static SimulationSubsystem simSub = new SimulationSubsystem();


  private static final double collectionDist = 0.75; //meters


  private StateHandler stateHandler = StateHandler.getInstance();

  public static synchronized SimulationSubsystem getInstance(){
      if (simSub == null){
        simSub = new SimulationSubsystem();
      }
      return simSub;
  }

  private class IntakeTimes{
    private static final double bb1OffTime = 0.1;
    private static final double bb2OnTime = 0.08;
    private static final double bb3OnTime = 0.2;
  }

  private class SourceIntakeTimes{
    private static final double bb3OnTime = 0.05;
    private static final double bb2OnTime = 0.1;
    private static final double bb4OffTime = 0.15;
  }

  private class ShootTimes{
    private static final double bb2OffTime = 0.04;
    private static final double bb3OffTime = 0.2;
    private static final double bb4OffTime = 0.3;
  }




  private ArrayList<Translation2d> notePoses;

  private boolean isCollecting = false;
  private Timer collectionTimer;

  private Timer shootTimer;

  private Timer babyBirdTimer;

  Integer[] list = {4 ,7, 11, 12, 13, 14, 15, 16};

  private List<Integer> filteredIDs = Arrays.asList(list);

  private ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");

  private double notesScored = 0;


  private Pose2d currentPose;


  private SuppliedValueWidget<Double> notes = driverDashboard.addNumber("NOTES SCORED", () -> notesScored).withPosition(4, 4);
  ;

  /** Creates a new SimulationSubsystem. */
  private SimulationSubsystem() {

    populateNotes();

    driverDashboard.addString("SIM ON", ()-> "SIM ON").withPosition(6, 4).withSize(3, 3);
    

   currentPose = new Pose2d();

   collectionTimer = new Timer();

   shootTimer = new Timer();

   babyBirdTimer = new Timer();

   stateHandler.bb2Covered = true;

   stateHandler.bb3Covered = true;



  }

  public void populateNotes(){
    notePoses = new ArrayList<>();

    notePoses.add(new Translation2d(2.89, 4.10)); //podiumLoc
    notePoses.add(new Translation2d(2.89, 5.56)); //midLoc
    notePoses.add(new Translation2d(2.89, 7.00)); //ampLoc

    // notePoses.add(new Translation2d(8.28, 7.45)); //1Loc
    // notePoses.add(new Translation2d(8.28, 5.78)); //2Loc
    notePoses.add(new Translation2d(8.28, 4.09)); //3Loc
    // notePoses.add(new Translation2d(8.28, 2.44)); //4Loc
    notePoses.add(new Translation2d(8.28,0.76)); //5Loc


  }

  /*
   * SETTING
   */
  public void updatePose(Pose2d robotPose){
    currentPose = robotPose;
  }

  public void setCollecting(boolean collecting){
    isCollecting = collecting;
  }


  
  /*
   * CALCULATING
   */
  public double simLLAngleToSpeaker(Pose2d aPose){

    Point allianceSpeaker = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue ? Constants.FieldConstants.blueSpeakerPos : Constants.FieldConstants.redSpeakerPos;


    return aPose.getRotation().getDegrees() - Math.toDegrees((Math.atan((aPose.getY() - allianceSpeaker.y)/(aPose.getX() - allianceSpeaker.x))));
  }

  public double getDistFromRobotToPose(Pose3d pose){
    Pose2d currPose = stateHandler.swervePose;

    double dist = pose.getTranslation().getDistance(new Translation3d(currPose.getX(), currPose.getY(), pose.getZ()));


    return Units.metersToInches(dist);
  }

  public boolean isInSource(){
    Pose2d currPose = stateHandler.swervePose;

    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
      

      double x = currPose.getX();
      double y = currPose.getY();





      return x>=FieldConstants.blueSourceStart.x 
          && y<=FieldConstants.blueSourceEnd.y;

    }
    else{

      double x = currPose.getX();
      double y = currPose.getY();

      return x<=FieldConstants.redSourceEnd.x 
          && y<=FieldConstants.redSourceStart.y;
    }

    
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (Utils.isSimulation()){

      updatePose(stateHandler.swervePose);


      //Intaking
      if (isCollecting){
          if (collectionTimer.get()==0){
          collectionTimer.start();
          stateHandler.bb1Covered = (true);
          }

          if (collectionTimer.get()>=IntakeTimes.bb1OffTime){
            stateHandler.bb1Covered = (false);
          }

          if (collectionTimer.get()>=IntakeTimes.bb2OnTime){
            stateHandler.bb2Covered = (true);
          }

          if (collectionTimer.get()>IntakeTimes.bb3OnTime){
            stateHandler.bb3Covered = (true);
            collectionTimer.stop();
            collectionTimer.reset();
            isCollecting = false;
        
          }
          
      }


    
      else if (stateHandler.currentIntakeArmState == IntakeArmStates.DEPLOYED && stateHandler.currentIntakeRollerState == IntakeRollerStates.INTAKE){

          for (int i = notePoses.size()-1; i >=0; i--){
            Translation2d notePos = notePoses.get(i);
            if (notePos.getDistance(currentPose.getTranslation())< collectionDist){


              isCollecting = true;

              notePoses.remove(i);
            }
          }
      }
      
      //Source Intaking
      if (stateHandler.currentArmState== ArmStates.BABY_BIRD 
      && stateHandler.currentShooterState == ShooterStates.BABY_BIRD_VELO
      && isInSource()){
        if (babyBirdTimer.get() ==0){
          babyBirdTimer.start();
          stateHandler.bb4Covered = (true);
        }

        if (babyBirdTimer.hasElapsed(SourceIntakeTimes.bb3OnTime)){
          stateHandler.bb3Covered = (true);
        }

        if (babyBirdTimer.hasElapsed(SourceIntakeTimes.bb2OnTime)){
          stateHandler.bb2Covered = (true);
        }

        if (babyBirdTimer.hasElapsed(SourceIntakeTimes.bb4OffTime)){
          stateHandler.bb4Covered = (false);
        }
      }
      else{
        babyBirdTimer.stop();
        babyBirdTimer.reset();
      }
      //Shooting
      if ( (stateHandler.currentShooterState == ShooterStates.SUBWOOFER_VELO || stateHandler.currentShooterState == ShooterStates.REVERSE_SUBWOOFER_VELO || stateHandler.currentShooterState == ShooterStates.RANGED_VELO || stateHandler.currentShooterState == ShooterStates.FRONT_AMP_VELO) && stateHandler.currentFeederState == FeederStates.FEED_TO_SHOOTER){

        if (shootTimer.get() == 0){
          shootTimer.start();
          stateHandler.bb4Covered = (true);
        }

        if (shootTimer.get() > ShootTimes.bb2OffTime){
          stateHandler.bb2Covered = (false);
        }

        if (shootTimer.get() > ShootTimes.bb3OffTime){
          stateHandler.bb3Covered = (false);
        }

        if (shootTimer.get() > ShootTimes.bb4OffTime){
          stateHandler.bb4Covered = (false);
          // notesScored++;
        }
      }
      else{
        shootTimer.stop();
        shootTimer.reset();
      }

      
    }

    SmartDashboard.putNumber("collecting timer", collectionTimer.get());

    SmartDashboard.putBoolean("isCollecting", isCollecting);

    SmartDashboard.putString("sim pose", currentPose.toString());
    

    SmartDashboard.putString("SIM SUBSYSTEM RUNNING", "!!!!!!");

    
    SmartDashboard.putBoolean("IS IN SOURCE", isInSource());

    // System.out.println(notesScored);
  }
}


// public int getSeenTagID(){
//   Pose2d currPose = stateHandler.getRobotPose();

//   int closestID = -1;
//   double closestDistance = Integer.MAX_VALUE;

//   AprilTag seven = LimelightInterface.getInstance().getAprilTagList().get(6);


//   if (!filteredIDs.contains(seven.ID)){
//       // System.out.println("failed on ID");

//     }

//     double angle = simLLAngleToPoint(seven.pose);
//     // System.out.println(angle);
//     if (angle == 100){
//       // System.out.println("Failed on angle");
//     }

//     double dist = seven.pose.getTranslation().getDistance(new Translation3d(currPose.getX(), currPose.getY(),seven.pose.getZ()));

//     closestID = seven.ID;

    
//     if (dist < closestDistance){
//       closestID = seven.ID;
//       closestDistance = dist;
//     }

  // for (AprilTag a : LimelightInterface.getInstance().getAprilTagList()){

  //   if (!filteredIDs.contains(a.ID)){
  //     continue;
  //   }

   
  //   double angle = simLLAngleToPoint(a.pose);

  //   if (angle == 100){
  //     continue;
  //   }

    

  //   double dist = a.pose.getTranslation().getDistance(new Translation3d(currPose.getX(), currPose.getY(),a.pose.getZ()));

    
  //   if (dist < closestDistance){
  //     closestID = a.ID;
  //     closestDistance = dist;
  //   }
  // }

//   return closestID;
// }

// public double simLLAngleToPoint(Pose3d aPose){

    


//   double theta = 0;


//   Rotation3d tagRotation = aPose.getRotation();

//   double rawHeading = Math.toDegrees(tagRotation.getZ());

//   if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
//     theta = Math.IEEEremainder(180+rawHeading, 360);
//   }
//   else{


//     theta = Math.IEEEremainder(rawHeading, 360);
//   }

//   // System.out.println(currentPose.getRotation().getDegrees() - theta);
//   return (Math.abs(currentPose.getRotation().getDegrees()-theta) < 30) ? currentPose.getRotation().getDegrees()-theta : 100;

  
  
// }
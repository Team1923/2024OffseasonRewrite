// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.autonutils;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class PathPlannerHelpers {


  /* PATH HELP */

  public static Command commandPathFrom(String pathName){

    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    if (path==null){
      return new InstantCommand();
    }
    return AutoBuilder.followPath(path);
  }


  /* PATHFINDING */
    public static Pose2d getLastPoseOf(PathPlannerPath path){
    
    List<Pose2d> poses = path.getPathPoses();

    Pose2d lastPose = poses.get(poses.size()-1);

    return new Pose2d(lastPose.getX(), lastPose.getY(), path.getGoalEndState().rotation());

  }
  public static Command pathfindToEndOfPath(String name){

    PathPlannerPath path = null;
    try {
      path = PathPlannerPath.fromPathFile(name);
    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    if (path==null){
      return new InstantCommand();
    }
    return AutoBuilder.pathfindToPose(getLastPoseOf(path), path.getGlobalConstraints());
  }



}

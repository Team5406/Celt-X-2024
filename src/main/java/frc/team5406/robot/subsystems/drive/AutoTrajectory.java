// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.subsystems.drive;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoTrajectory {
  DriveSubsystem m_driveSubsystem;
  Command m_swerveCommand;
  Pair<String,List<PathPlannerPath>> m_auto;

  /**
   * Creates new path trajectory using a physical x,y coordinate points
   * @param driveSubsystem DriveSubsystem required for drivetrain movement
   * @param waypoints List of x, y coordinate pairs in trajectory
   * @param pathConstraints Path following constraints
   */
  public AutoTrajectory(DriveSubsystem driveSubsystem, List<Pose2d> waypoints, PathConstraints pathConstraints, GoalEndState end) {
    this.m_driveSubsystem = driveSubsystem;

    
    // Generate path from waypoints
    m_auto = new Pair<String, List<PathPlannerPath>>("", List.of(new PathPlannerPath(
      PathPlannerPath.bezierFromPoses(waypoints),
      pathConstraints,
      end
    )));
        /*Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red){
            m_auto.getSecond().get(0).flipPath();
      }else{
        m_auto.getSecond();
      }
    }*/

  }

  /** Return initial pose */
  public Pose2d getInitialPose() {
    return m_auto.getSecond().get(0).getPreviewStartingHolonomicPose();
  }

  /**
   * Get auto command to execute path
   * @return Auto command group that will stop when complete
   */
  public Command getCommand() {
    Command autoCommand = m_auto.getSecond().size() == 1
      ? AutoBuilder.followPath(m_auto.getSecond().get(0))
      : new PathPlannerAuto(m_auto.getFirst());

    return autoCommand
      .andThen(() -> m_driveSubsystem.resetRotatePID())
      .andThen(m_driveSubsystem.stopCommand())
      .andThen(m_driveSubsystem.lockCommand());
  }
}
package frc.team5406.robot.autos;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.subsystems.drive.AutoTrajectory;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;

public class RedDriveStraight extends SequentialCommandGroup {
  /** Creates a new Leave. */
  final DriveSubsystem drive;
  public RedDriveStraight(DriveSubsystem driveSubsystem) {
    this.drive = driveSubsystem;
    List<Pose2d> path = List.of(
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
      new Pose2d(new Translation2d(3, 0.0), Rotation2d.fromDegrees(0.0))
    );
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
        GoalEndState end = new GoalEndState(0.0,  Rotation2d.fromDegrees(0));
    AutoTrajectory auto = new AutoTrajectory(drive, path, constraints, end);

    addCommands(
      new InstantCommand(() -> drive.resetPose(new Pose2d(new Translation2d(16.4, 0.0), Rotation2d.fromDegrees(180.0)))),
      new WaitCommand(3),
      auto.getCommand()
    );
  }
}

package frc.team5406.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team5406.robot.subsystems.drive.AutoTrajectory;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;

public class DriveStraight extends SequentialCommandGroup {
  /** Creates a new Leave. */
  public DriveStraight(DriveSubsystem driveSubsystem) {
    List<Pose2d> path = List.of(
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0)),
      new Pose2d(new Translation2d(1, 0.0), Rotation2d.fromDegrees(0.0))
    );
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    AutoTrajectory auto = new AutoTrajectory(driveSubsystem, path, constraints);

    addCommands(
      auto.getCommand()
    );
  }
}

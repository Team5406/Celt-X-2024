package frc.team5406.robot.autos;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.commands.ShootFromSubwoofer;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.drive.AutoTrajectory;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;

public class OneGamePiece extends SequentialCommandGroup {
  /** Creates a new Leave. */
    final ArmSubsystem arm;
    final ShooterSubsystem shooter;
    final FeederSubsystem feeder;
    final DriveSubsystem drive;
    
  
    
  public OneGamePiece(DriveSubsystem drive, ArmSubsystem arm, ShooterSubsystem shooter, FeederSubsystem feeder) {
    this.arm = arm;
    this.shooter = shooter;
    this.feeder = feeder;
    this.drive = drive;
    
    List<Pose2d> path = List.of(
      new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(60.0)),
      new Pose2d(new Translation2d(-2.7,-2.9), Rotation2d.fromDegrees(0.0)),
      new Pose2d(new Translation2d(-7.1, -2.9), Rotation2d.fromDegrees(0.0))

    );
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    AutoTrajectory auto = new AutoTrajectory(drive, path, constraints);

    addCommands(
    new ParallelDeadlineGroup(
      new WaitCommand(4),
      new ParallelCommandGroup(
        new ShootFromSubwoofer(arm, shooter, false), 
        new SequentialCommandGroup(
          new WaitCommand(1), 
          new Shoot(feeder)
        )
      )
    ),
    auto.getCommand()
    );
  }
}

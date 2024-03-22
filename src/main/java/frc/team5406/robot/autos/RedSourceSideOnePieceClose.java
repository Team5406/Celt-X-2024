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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.commands.ShootFromSubwoofer;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.drive.AutoTrajectory;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;

public class RedSourceSideOnePieceClose extends SequentialCommandGroup {
  /** Creates a new Leave. */
    final ArmSubsystem arm;
    final ShooterSubsystem shooter;
    final FeederSubsystem feeder;
    final DriveSubsystem drive;
    
  
    
  public RedSourceSideOnePieceClose(DriveSubsystem drive, ArmSubsystem arm, ShooterSubsystem shooter, FeederSubsystem feeder) {
    this.arm = arm;
    this.shooter = shooter;
    this.feeder = feeder;
    this.drive = drive;
    
    List<Pose2d> path = List.of(
      new Pose2d(new Translation2d(0.7, 4.3), Rotation2d.fromDegrees(-60.0)),
      new Pose2d(new Translation2d(2.2,2.5), Rotation2d.fromDegrees(-60.0)),
      new Pose2d(new Translation2d(5.5, 1.3), Rotation2d.fromDegrees(0.0))

    );
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 4 * Math.PI, 2 * Math.PI);
    GoalEndState end = new GoalEndState(0.0,  Rotation2d.fromDegrees(0));
    AutoTrajectory auto = new AutoTrajectory(drive, path, constraints, end);

    addCommands(
    new InstantCommand(() -> drive.resetPose(new Pose2d(Constants.FIELD_WIDTH-0.7,4.3, Rotation2d.fromDegrees(240.0)))),
    new ParallelDeadlineGroup(
        new WaitCommand(2),
        new ShootFromSubwoofer(arm, shooter)
    ),
    new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new Shoot(feeder)
    ),
    new ParallelCommandGroup(
      new InstantCommand(()-> shooter.stopShooter(), shooter),
      new InstantCommand(()-> feeder.stopFeeder(), feeder),
      new ParallelRaceGroup(
        new WaitCommand(2),
        new RunCommand(()->arm.gotoArmAngle(Constants.ArmHardware.ARM_ZERO_ANGLE), arm)
      ),
      auto.getCommand()
    )
    );
  }
}
package frc.team5406.robot.autos;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTag;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.commands.AlignWithSpeaker;
import frc.team5406.robot.commands.AlignWithTag;
import frc.team5406.robot.commands.AlignWithTagAuto;
import frc.team5406.robot.commands.IntakeFromFloor;
import frc.team5406.robot.commands.MoveArmTrapezoidFinished;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.commands.ShootFromSubwoofer;
import frc.team5406.robot.commands.SpinUp;
import frc.team5406.robot.commands.SpinUpFinished;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.drive.AutoTrajectory;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.subsystems.vision.VisionSubsystem;

public class BlueAmpSideThreePieceOneFar extends SequentialCommandGroup {
  /** Creates a new Leave. */
    final ArmSubsystem arm;
    final ShooterSubsystem shooter;
    final FeederSubsystem feeder;
    final DriveSubsystem drive;
    final VisionSubsystem camera;
    final IntakeSubsystem intake;
    private final Supplier<AprilTag> targetSupplier;
  
  public BlueAmpSideThreePieceOneFar(DriveSubsystem drive, ArmSubsystem arm, ShooterSubsystem shooter, FeederSubsystem feeder, VisionSubsystem camera, IntakeSubsystem intake, Supplier<AprilTag> targetSupplier) {
    this.arm = arm;
    this.shooter = shooter;
    this.feeder = feeder;
    this.drive = drive;
    this.camera = camera;
    this.intake = intake;
    this.targetSupplier = targetSupplier;
    
    List<Pose2d> path = List.of(
      new Pose2d(new Translation2d(0.81, 6.66), Rotation2d.fromDegrees(0.0)),
      new Pose2d(new Translation2d(2.7, 6.9), Rotation2d.fromDegrees(20))
    );
    GoalEndState end = new GoalEndState(0.0,  Rotation2d.fromDegrees(30), true);

    List<Pose2d> path2 = List.of(
      new Pose2d(new Translation2d(2.7, 6.9), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(5.2, 6.9), Rotation2d.fromDegrees(0)),
      new Pose2d(new Translation2d(8.2, 5.9), Rotation2d.fromDegrees(-140)),
      new Pose2d(new Translation2d(5.2, 6.9), Rotation2d.fromDegrees(150))
    );
    GoalEndState end2 = new GoalEndState(0.0,  Rotation2d.fromDegrees(15), true);

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 4 * Math.PI, 4 * Math.PI);
    AutoTrajectory auto1 = new AutoTrajectory(drive, path, constraints, end);
    AutoTrajectory auto2 = new AutoTrajectory(drive, path2, constraints, end2);

    addCommands(
        new InstantCommand(() -> drive.resetPose(new Pose2d(0.81, 6.66, Rotation2d.fromDegrees(60.0)))),
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(2),
                    new ShootFromSubwoofer(arm, shooter)
                ),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5),
                    new Shoot(feeder)
                ),
                new ParallelDeadlineGroup(
                    auto1.getCommand(),
                    new IntakeFromFloor(feeder, intake),
                    new SpinUpFinished(shooter, Constants.ShooterHardware.SHOOTER_TARGET_RPM, false),
                    new MoveArmTrapezoidFinished(33.4, arm)
                ),
                new ParallelRaceGroup(
                    new WaitCommand(1.5),
                    new ParallelCommandGroup(
                      new AlignWithTagAuto(camera, drive, targetSupplier),
                      new AlignWithSpeaker(arm, camera, shooter, targetSupplier)
                    )
                ),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5),
                    new Shoot(feeder)
                ),
                new ParallelDeadlineGroup(
                    auto2.getCommand(),
                    new IntakeFromFloor(feeder, intake)
                ),
                new ParallelRaceGroup(
                    new WaitCommand(1.5),
                    new ParallelCommandGroup(
                      new AlignWithTagAuto(camera, drive, targetSupplier),
                      new AlignWithSpeaker(arm, camera, shooter, targetSupplier)
                    )
                ),
                new ParallelDeadlineGroup(
                    new WaitCommand(0.5),
                    new Shoot(feeder)
                )
            )
        )
    );
  }

}

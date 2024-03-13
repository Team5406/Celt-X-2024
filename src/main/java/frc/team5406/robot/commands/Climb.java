package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.TrapSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class Climb extends SequentialCommandGroup{
  public Climb(ArmSubsystem arm, TrapSubsystem trap, ShooterSubsystem shoot, FeederSubsystem feed, ClimbSubsystem climb){
    addCommands(
        new ParallelCommandGroup(
          new InstantCommand(() -> arm.climbing()),
          new MoveClimber(climb, 115, 2),
          new SequentialCommandGroup(
                new WaitCommand(0.18),
                new MoveArmTrapezoidFinished(90, arm)
          ),
          new MoveTrapTrapezoidFinished(21.25, trap)
        ),
        new WaitCommand(0.5),
        new ParallelCommandGroup(
          new MoveClimber(climb, Constants.ClimberHardware.CLIMBER_CLIMB_HEIGHT, 5),
          new MoveArmTrapezoidFinished(120, arm),
          new SpinUpFinished(shoot, 1700, true)
        ),
        new ParallelDeadlineGroup(
          new WaitCommand(0.5),
          new Shoot(feed)
        )
    );
   }
}
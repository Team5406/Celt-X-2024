
package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.TrapSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class Unclimb extends SequentialCommandGroup{

  public Unclimb(ArmSubsystem arm, TrapSubsystem trap, ShooterSubsystem shoot, FeederSubsystem feed, ClimbSubsystem climb){
    addCommands(
      new ParallelCommandGroup(
        new MoveClimber(climb, Constants.ClimberHardware.CLIMBER_MAX_HEIGHT, 2),
        new SequentialCommandGroup(
          new WaitCommand(0.18),
          new MoveArmTrapezoidFinished(Constants.ArmHardware.ARM_ZERO_ANGLE, arm)
          ),
          new MoveTrapTrapezoidFinished(Constants.TrapHardware.TRAP_ZERO_ANGLE, trap)
          ),
      new InstantCommand(() -> arm.notClimbing())
    );
  }
}



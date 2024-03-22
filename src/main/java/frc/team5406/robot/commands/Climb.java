
package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.TrapSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class Climb extends SequentialCommandGroup{

  public Climb(ArmSubsystem arm, TrapSubsystem trap, ShooterSubsystem shoot, FeederSubsystem feed, ClimbSubsystem climb)
  {
      
    addCommands(
        new ParallelCommandGroup(
          new InstantCommand(() -> arm.climbing()),
          new MoveClimber(climb, 115, 2),
          new SequentialCommandGroup(
              new WaitCommand(.2),
                      new MoveArmTrapezoidFinished(90, arm)
          ),
         new MoveTrapTrapezoidFinished(38.25, trap)
        ),
        new WaitCommand(0.5),
          new MoveClimber(climb, Constants.ClimberHardware.CLIMBER_CLIMB_HEIGHT, 10),
        new ParallelDeadlineGroup(
          new WaitCommand(2),
          new MoveArmTrapezoidFinished(130, arm),
          new SpinUpFinished(shoot, 750, true)
        ),
        new WaitCommand(0.5),  
        new ParallelDeadlineGroup(
          new WaitCommand(1.5),
          new Shoot(feed)
        ),
        new MoveArmTrapezoidFinished(130, arm),
        new WaitCommand(5)

    );


    
   }

   
}



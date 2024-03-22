
package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class AmpShot extends SequentialCommandGroup{

  public AmpShot(ArmSubsystem arm, ShooterSubsystem shooter, FeederSubsystem feeder)
  {
      
    addCommands(
      new ParallelCommandGroup(
        new MoveArmTrapezoidFinished(Constants.AmpShotSettings.AMPSHOT_ARM_ANGLE, arm),
        new SpinUpFinished(shooter, Constants.AmpShotSettings.AMPSHOT_SET_SPEED, true)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new Shoot(feeder)
      )
    );
   }
}



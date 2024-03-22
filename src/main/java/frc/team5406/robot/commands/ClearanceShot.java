package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class ClearanceShot extends SequentialCommandGroup {
    
    public ClearanceShot(ArmSubsystem arm, ShooterSubsystem shooter, FeederSubsystem feeder, boolean backwards)
  {
    addCommands(
      new ParallelCommandGroup(
        new MoveArmTrapezoidFinished(Constants.ClearanceSettings.CLEARANCE_ARM_ANGLE, arm),
        new SpinUpFinished(shooter, Constants.ClearanceSettings.CLEARANCE_SET_SPEED, true)
      ),
      new ParallelDeadlineGroup(
        new WaitCommand(0.5),
        new Shoot(feeder)
      )
    );
  }
}
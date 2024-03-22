
package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;

public class ShootFromSubwoofer extends ParallelCommandGroup {

    public ShootFromSubwoofer(ArmSubsystem arm, ShooterSubsystem shooter)
  {     
    addCommands(
      new MoveArmTrapezoidFinished(Constants.FixedShotSettings.SUBWOOFER_ARM_ANGLE, arm),
      new SpinUpFinished(shooter, 2998, false)
    );
  }
}
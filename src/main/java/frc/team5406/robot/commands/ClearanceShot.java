package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;

public class ClearanceShot extends ParallelCommandGroup {
    public ClearanceShot(ArmSubsystem arm, ShooterSubsystem shooter, boolean backwards){
    addCommands(
      new MoveArmTrapezoidFinished(Constants.ClearanceSettings.CLEARANCE_ARM_ANGLE, arm),
      new RunCommand(() -> shooter.setShooterSpeed(Constants.ClearanceSettings.CLEARANCE_SET_SPEED), shooter)
    );
  }
}
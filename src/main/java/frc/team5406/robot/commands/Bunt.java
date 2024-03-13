package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;

public class Bunt extends ParallelCommandGroup {
    public Bunt(ArmSubsystem arm, ShooterSubsystem shooter, boolean backwards){
    addCommands(
      new MoveArmTrapezoid(Constants.BuntSettings.BUNT_ARM_ANGLE, arm),
      new RunCommand(() -> shooter.setShooterSpeed(Constants.BuntSettings.BUNT_SET_SPEED), shooter)
    );
  }
}
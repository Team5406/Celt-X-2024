package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class ShooterOuttake extends ParallelCommandGroup {
    
    public ShooterOuttake(FeederSubsystem feeder, IntakeSubsystem intake, ShooterSubsystem shooter){
      addCommands(
        new RunCommand(() -> intake.setIntakeSpeed(Constants.IntakeHardware.OUTTAKE_TARGET_RPM), intake),
        new RunCommand(() -> feeder.setFeederSpeed(Constants.FeederHardware.OUTTAKE_FEEDER_SPEED), feeder),
        new RunCommand(() -> shooter.setShooterSpeed(Constants.ShooterHardware.SHOOTER_OUTTAKE_SPEED), shooter)
      );
    }
}

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;

public class Outtake extends ParallelCommandGroup {
    public Outtake(FeederSubsystem feeder, IntakeSubsystem intake){
      addCommands(
        new RunCommand(() -> intake.setIntakeSpeed(Constants.IntakeHardware.OUTTAKE_TARGET_RPM), intake),
        new RunCommand(() -> feeder.setFeederSpeed(Constants.FeederHardware.OUTTAKE_FEEDER_SPEED), feeder)
      );  
    }
}
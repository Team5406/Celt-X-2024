

package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;

public class IntakeFromFloor extends ParallelCommandGroup {
    
    public IntakeFromFloor(FeederSubsystem feeder, IntakeSubsystem intake){
      addCommands(
        //new RunCommand(() -> intake.setIntakeSpeed(Constants.IntakeHardware.INTAKE_TARGET_RPM), intake)
        new RunCommand(() -> intake.set(1.0), intake)
        .unless(() -> feeder.haveNote() ),
        new Feed(feeder)
      );  
    }
}
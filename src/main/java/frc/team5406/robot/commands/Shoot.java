package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class Shoot extends Command {
    
    boolean noteSeen = false;
    final FeederSubsystem feeder;

    public Shoot(FeederSubsystem feeder)
  {
    this.feeder = feeder;
    addRequirements(feeder);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    feeder.setFeederSpeed(-Constants.FeederHardware.FEEDER_TARGET_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(feeder.getToFStatus() && feeder.getToFDistance() < Constants.FeederHardware.TOF_NOTE_SEEN_THRESHOLD){
      noteSeen = true;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  //Math.abs(arm.getArmVelocity()) < 500
  {
    return noteSeen && feeder.getToFStatus() && feeder.getToFDistance() >= Constants.FeederHardware.TOF_NOTE_PASSED_THRESHOLD;
}

}
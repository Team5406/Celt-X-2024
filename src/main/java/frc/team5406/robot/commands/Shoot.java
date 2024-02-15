package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class Shoot extends Command {
    
    
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
    feeder.setFeederSpeed(Constants.SubwooferSettings.SHOOT_FEEDER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  //Math.abs(arm.getArmVelocity()) < 500
  {
    return false;
    /*if(Math.abs(arm.getArmAngle() - Constants.SUBWOOFER_ARM_ANGLE) <10 && Math.abs(arm.getArmVelocity()) < 1500){
      return true;
    }else {
      return false;
    } */ }
}
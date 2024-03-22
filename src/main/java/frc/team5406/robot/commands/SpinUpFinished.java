package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class SpinUpFinished extends Command {
    
    boolean noteSeen = false;
    final ShooterSubsystem shoot;
    final double speed;
    final boolean same;

    public SpinUpFinished(ShooterSubsystem shoot, double speed, boolean same)
  {
    this.shoot = shoot;
    this.speed = speed;
    this.same = same;
    addRequirements(shoot);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
          shoot.setShooterSpeed(speed, same);
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  //Math.abs(arm.getArmVelocity()) < 500
  {
    return Math.abs(speed - shoot.getShooterSpeedLeft())< Constants.ShooterHardware.SPEED_THRESHOLD;
}

}
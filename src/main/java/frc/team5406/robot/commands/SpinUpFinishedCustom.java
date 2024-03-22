package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class SpinUpFinishedCustom extends Command {
    
    boolean noteSeen = false;
    final ShooterSubsystem shoot;
    final double left;
    final double right;

    public SpinUpFinishedCustom(ShooterSubsystem shoot, double left, double right)
  {
    this.shoot = shoot;
    this.left = left;
    this.right = right;
    addRequirements(shoot);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
          shoot.setShooterSpeeds(left, right);
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
    return Math.abs(left - shoot.getShooterSpeedLeft())< Constants.ShooterHardware.SPEED_THRESHOLD && Math.abs(right - shoot.getShooterSpeedRight())< Constants.ShooterHardware.SPEED_THRESHOLD;
}

}
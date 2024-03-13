package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class SpinUp extends Command {  
    boolean noteSeen = false;
    double speed = 0;
    double angle = 0;
    final FeederSubsystem feeder;
    final ShooterSubsystem shooter;

    public SpinUp(FeederSubsystem feeder, ShooterSubsystem shooter, double speed){
      this.feeder = feeder;
      this.shooter = shooter;
      this.speed = speed;
      addRequirements(shooter);
      addRequirements(feeder);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    feeder.setFeederSpeed(0);
    shooter.setShooterSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    shooter.setShooterSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return false;
  }
}
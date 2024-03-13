package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.subsystems.ArmSubsystem;

public class GoToAngle extends Command {
    
    final ArmSubsystem arm;
    final Double angle;

    public GoToAngle(ArmSubsystem arm, double angle){
    this.arm = arm;
    this.angle = angle;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    arm.gotoArmAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    System.out.println("end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    if(Math.abs(arm.getArmAngle() - angle) <10 && Math.abs(arm.getArmVelocity()) < 1500){
      return true;
    }else {
      return false;
    }  }
}
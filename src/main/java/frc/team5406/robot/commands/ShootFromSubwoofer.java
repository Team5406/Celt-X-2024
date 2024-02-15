
package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;

public class ShootFromSubwoofer extends Command {
    
    final ArmSubsystem arm;
    final ShooterSubsystem shooter;
    final boolean backwards;

    public ShootFromSubwoofer(ArmSubsystem arm, ShooterSubsystem shooter, boolean backwards)
  {
    this.arm = arm;
    this.shooter = shooter;
    this.backwards = backwards;
    addRequirements(arm);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    if (backwards){
      arm.gotoArmAngle(Constants.SubwooferSettings.SUBWOOFER_ARM_ANGLE_BACKWARDS);
      shooter.setShooterSpeed(Constants.SubwooferSettings.SUBWOOFER_SET_SPEED);
    }else {
      arm.gotoArmAngle(Constants.SubwooferSettings.SUBWOOFER_ARM_ANGLE);
      shooter.setShooterSpeed(Constants.SubwooferSettings.SUBWOOFER_SET_SPEED);
    }
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
   /*  if(Math.abs(arm.getArmAngle() - Constants.SUBWOOFER_ARM_ANGLE) <10 && Math.abs(arm.getArmVelocity()) < 1500){
      return true;
    }else {
      return false;
    }  
  */}
}
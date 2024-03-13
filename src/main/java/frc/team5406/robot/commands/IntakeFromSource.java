package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;

public class IntakeFromSource extends Command {
    final ArmSubsystem arm;
    final ShooterSubsystem shooter;
    final FeederSubsystem feeder;

    public IntakeFromSource(ArmSubsystem arm, ShooterSubsystem shooter, FeederSubsystem feeder){
      this.arm = arm;
      this.shooter = shooter;
      this.feeder = feeder;
      addRequirements(feeder);
      addRequirements(arm);
      addRequirements(shooter);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    arm.gotoArmAngle(Constants.SourceIntakeHardware.INTAKE_SET_ANGLE);
    shooter.setShooterSpeed(Constants.SourceIntakeHardware.INTAKE_SET_SPEED);
    feeder.setFeederSpeed(Constants.SourceIntakeHardware.INTAKE_FEEDER_SPEED);
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
    return false;
   }
}
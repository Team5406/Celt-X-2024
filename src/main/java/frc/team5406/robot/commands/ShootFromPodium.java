package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;

public class ShootFromPodium extends ParallelCommandGroup { 
  public ShootFromPodium(ArmSubsystem arm, ShooterSubsystem shooter, boolean backwards){
    Command armTraj;
    if(backwards){
      armTraj = new RunCommand(() -> arm.gotoArmAngle(Constants.PodiumSettings.PODIUM_ARM_ANGLE_BACKWARDS), arm);
    }else{
      armTraj = new RunCommand(() -> arm.gotoArmAngle(Constants.PodiumSettings.PODIUM_ARM_ANGLE), arm);
    }
      
    addCommands(
      armTraj,
      new RunCommand(() -> shooter.setShooterSpeed(Constants.PodiumSettings.PODIUM_SET_SPEED), shooter)
    );

  }
}
package frc.team5406.robot.commands;

import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class MoveArmTrapezoidFinished extends Command {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

   private final ArmSubsystem arm;
   private final double position;
 
  public MoveArmTrapezoidFinished(double position, ArmSubsystem arm) {
    this.position = position;
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // End when the controller is at the reference.
        arm.gotoArmAngle(position); 
  }

  @Override
  public void execute() {
    //arm.gotoArmAngle(position);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return Math.abs(arm.getArmVelocity()) < 50 && Math.abs(arm.getArmAngle()-position) < (Constants.ArmHardware.ARM_POSITION_TOLERANCE*1.1*(Constants.DEGREES_PER_ROTATION / Constants.ArmHardware.ARM_GEAR_RATIO));

  }

  @Override
  public void end(boolean interrupted){
    
  }
}
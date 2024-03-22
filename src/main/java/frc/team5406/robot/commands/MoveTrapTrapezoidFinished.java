package frc.team5406.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AddressableLED;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.TrapSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class MoveTrapTrapezoidFinished extends Command {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

   private final TrapSubsystem trap;
   private final double position;
 
  public MoveTrapTrapezoidFinished(double position, TrapSubsystem trap) {
    this.position = position;
    this.trap = trap;
    addRequirements(trap);
  }

  @Override
  public void initialize() {
    // End when the controller is at the reference.
        trap.goToTrapAngle(position); 
  }

  @Override
  public void execute() {
    //arm.gotoArmAngle(position);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return Math.abs(trap.getTrapVelocity()) < 50 && Math.abs(trap.getTrapAngle()-position) < (Constants.TrapHardware.TRAP_POSITION_TOLERANCE*1.1);

  }

  @Override
  public void end(boolean interrupted){
    
  }
}
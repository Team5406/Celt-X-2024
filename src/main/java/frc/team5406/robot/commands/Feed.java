package frc.team5406.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.FeederSubsystem;

/** A command that will turn the robot to the specified angle. */
public class Feed extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */

  private final FeederSubsystem feeder;

  public Feed(FeederSubsystem feeder) {
    super(
        new PIDController(Constants.FeederHardware.FEEDER_SPIN_P, Constants.FeederHardware.FEEDER_SPIN_I, Constants.FeederHardware.FEEDER_SPIN_D),
        // Close loop on heading
        feeder::getToFDistance, //FIXME check status + loop rate
        // Set reference to target
        Constants.FeederHardware.FEEDER_NOTE_SETPOINT,
        // Pipe output to turn robot
        output -> feeder.setFeederSpeed(output),
        // Require the drive
        feeder);

    // Set the controller to be continuous (because it is an angle controller)
    //getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(Constants.FeederHardware.FEEDER_SETPOINT_TOLERANCE);

    this.feeder = feeder;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }
}
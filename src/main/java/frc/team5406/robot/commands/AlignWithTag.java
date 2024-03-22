

package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;

public class AlignWithTag extends Command {
    
    final DriveSubsystem drive;
    final DoubleSupplier vX;
    final DoubleSupplier vY;
    final DoubleSupplier vRotate;
    final VisionSubsystem camera;
    final PIDController turnPID = new PIDController(0.025,0, 0);
    private final  Supplier<AprilTag> targetSupplier;

    private int tagID;

    public AlignWithTag(VisionSubsystem camera, DriveSubsystem drive, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRotate, Supplier<AprilTag> targetSupplier)
  {
    this.turnPID.setTolerance(0.5);
    turnPID.setSetpoint(Constants.VisionHardware.SPEAKER_OFFSET);
    turnPID.enableContinuousInput(-180, 180);
    this.drive = drive;
    this.camera = camera;
    this.vX = vX;
    this.vY = vY;
    this.vRotate = vRotate;
    this.targetSupplier = targetSupplier;
    this.addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    turnPID.reset();
    tagID = targetSupplier.get().ID;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    
    if (camera.hasTarget(tagID)){
        camera.getYaw(tagID).ifPresent((yaw) -> {
            //System.out.println(yaw);
            double output = turnPID.calculate(yaw.doubleValue());
            double ff = Math.signum(output)*0.25;
            drive.driveCommand(vX, vY, () -> -MathUtil.clamp(output + ff, -1, 1)).execute();
        }
    );
    } else {
        drive.driveCommand(vX, vY, vRotate).execute();
    }
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
    /*if(Math.abs(arm.getArmAngle() - Constants.SUBWOOFER_ARM_ANGLE) <10 && Math.abs(arm.getArmVelocity()) < 1500){
      return true;
    }else {
      return false;
    } */ }
}
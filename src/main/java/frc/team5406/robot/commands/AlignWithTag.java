package frc.team5406.robot.commands;

import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.subsystems.vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignWithTag extends Command {
    final DriveSubsystem drive;
    final DoubleSupplier vX;
    final DoubleSupplier vY;
    final DoubleSupplier vRotate;
    final VisionSubsystem camera;
    final PIDController turnPID = new PIDController(0.05,0, 0);
    final int tagID;

    public AlignWithTag(VisionSubsystem camera, DriveSubsystem drive, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRotate, int tagID){
    this.turnPID.setTolerance(2);
    turnPID.setSetpoint(0);
    turnPID.enableContinuousInput(-180, 180);
    this.drive = drive;
    this.camera = camera;
    this.vX = vX;
    this.vY = vY;
    this.vRotate = vRotate;
    this.tagID = tagID;
    this.addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    turnPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if (camera.hasTarget(tagID)){
        camera.getYaw(tagID).ifPresent((yaw) -> {
            System.out.println(yaw);
            drive.driveCommand(vX, vY, () -> -MathUtil.clamp(turnPID.calculate(yaw.doubleValue()), -1, 1)).execute();
        }
    );
    } else {
        drive.driveCommand(vX, vY, vRotate).execute();
    }
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
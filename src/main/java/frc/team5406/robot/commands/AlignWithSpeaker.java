package frc.team5406.robot.commands;

import frc.team5406.robot.Constants;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.subsystems.vision.VisionSubsystem;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AlignWithSpeaker extends Command {
    private final ArmSubsystem arm;
    private final VisionSubsystem camera;
    private final ShooterSubsystem shoot;
    private final  Supplier<AprilTag> targetSupplier;
    private int tagID;

  private final ProfiledPIDController armProfile = new ProfiledPIDController(Constants.ArmHardware.ARM_PID_PROFILED_P,
  Constants.ArmHardware.ARM_PID_PROFILED_I,
  Constants.ArmHardware.ARM_PID_PROFILED_D,
  new TrapezoidProfile.Constraints(
      Constants.ArmHardware.ARM_MAX_SPEED,
      Constants.ArmHardware.ARM_MAX_ACCELERATION));

    public AlignWithSpeaker(ArmSubsystem arm, VisionSubsystem camera, ShooterSubsystem shoot, Supplier<AprilTag> targetSupplier){
        this.arm = arm;
        this.camera = camera;
        this.shoot = shoot;
        this.targetSupplier = targetSupplier;
        this.addRequirements(arm);
        this.addRequirements(shoot);
    }

    @Override
    public void initialize(){
      tagID = targetSupplier.get().ID;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        if (camera.hasTarget(tagID)){
          camera.getPitch(tagID).ifPresent((pitch) -> {
            double angle = MathUtil.clamp(0.777 * pitch +44.1, Constants.ArmHardware.ARM_ZERO_ANGLE, 130);
            double turnOutput = armProfile.calculate(arm.getArmAngle(), angle);
            arm.useOutputPosition(turnOutput, armProfile.getSetpoint());
          });
        }
        shoot.setShooterSpeed(3000);
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

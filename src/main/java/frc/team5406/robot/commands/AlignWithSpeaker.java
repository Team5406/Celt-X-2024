package frc.team5406.robot.commands;

import frc.team5406.robot.Constants;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.team5406.robot.subsystems.vision.VisionSubsystem;
import frc.team5406.robot.subsystems.vision.AprilTagCamera;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.commands.MoveArmTrapezoid;
import frc.team5406.robot.subsystems.ShooterSubsystem;

public class AlignWithSpeaker extends Command {
    private final ArmSubsystem arm;
    private final VisionSubsystem camera;
    private final ShooterSubsystem shoot;
    private final  Supplier<AprilTag> targetSupplier;
    private int tagID;


    public AlignWithSpeaker(ArmSubsystem arm, VisionSubsystem camera, ShooterSubsystem shoot, Supplier<AprilTag> targetSupplier){
        this.arm = arm;
        this.camera = camera;
        this.shoot = shoot;
        this.targetSupplier = targetSupplier;
        this.addRequirements(arm);
        this.addRequirements(shoot);
    }


    @Override
    public void initialize()
    {
      tagID = targetSupplier.get().ID;
      /*if (Constants.currentAlliance != null) {
        if(Constants.currentAlliance.get() == Alliance.Blue){
           tagID = 8;
        }else{
          tagID = 7;
        }
      }*/

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
        if (camera.hasTarget(tagID)){
          camera.getPitch(tagID).ifPresent((pitch) -> {
            double angle = MathUtil.clamp(0.8157*pitch + 40.676, Constants.ArmHardware.ARM_ZERO_ANGLE, 125);
            arm.gotoArmAngle(angle);
          });
        }
        shoot.setShooterSpeed(Constants.ShooterHardware.SHOOTER_TARGET_RPM);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
      return false;
    }


}

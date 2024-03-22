package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;

public class ArmZero extends Command {

    private final ArmSubsystem arm;
    private int currentCounter = 0;
    public ArmZero(ArmSubsystem arm) {
        this.arm = arm;
   //     addRequirements(shooter);
    }

    @Override
    public void initialize() {
       // ShooterSubsystem.disableHoodLimits();
        currentCounter = 0;
    }

    @Override
    public void execute() {
        arm.setArmSpeed(Constants.ArmHardware.ARM_ZEROING_SPEED); // 1.65 is magic number
        if(arm.getArmCurrent() >= Constants.ArmHardware.CURRENT_SPIKE){
            currentCounter++;
        }
    }
    @Override
    public boolean isFinished() {
        //return ShooterSubsystem.getHoodCurrent() >= Constants.NEO550_CURRENT_SPIKE && Math.abs(ShooterSubsystem.getHoodVelocity()) <= 10;
        return currentCounter >= 3;
    }

    @Override
    public void end(boolean interrupted){
        arm.resetArmAngle();
        arm.stopArm();
        //arm.enableHoodLimits();
    }
}
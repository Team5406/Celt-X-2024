package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.TrapSubsystem;

public class TrapZero extends Command {

    private final TrapSubsystem trap;
    private int currentCounter = 0;
    public TrapZero(TrapSubsystem trap) {
        this.trap = trap;
    }

    @Override
    public void initialize() {
        currentCounter = 0;
    }

    @Override
    public void execute() {
        trap.setTrapSpeed(Constants.TrapHardware.TRAP_ZEROING_SPEED); // magic number
        if(trap.getTrapCurrent() >= Constants.TrapHardware.CURRENT_SPIKE){ 
            currentCounter++;
        }
    }
    @Override
    public boolean isFinished() {
        return currentCounter >= 3;
    }

    @Override
    public void end(boolean interrupted){
        trap.resetTrapAngle();
        trap.stopTrap();
    }
}

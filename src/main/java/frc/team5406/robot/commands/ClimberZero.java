package frc.team5406.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class ClimberZero extends Command {

    private final ClimbSubsystem climber;
    private int currentCounter = 0;
    public ClimberZero(ClimbSubsystem climber) {
        this.climber = climber;
    addRequirements(climber);
    }

    @Override
    public void initialize() {
       // ShooterSubsystem.disableHoodLimits();
        currentCounter = 0;
    }

    @Override
    public void execute() {
        climber.setClimberSpeed(Constants.ClimberHardware.CLIMBER_ZEROING_SPEED); // 1.65 is magic number
        if(climber.getClimberCurrent() >= Constants.ClimberHardware.CURRENT_SPIKE){
            currentCounter++;
        }
    }
    @Override
    public boolean isFinished() {
        //return ShooterSubsystem.getHoodCurrent() >= Constants.NEO550_CURRENT_SPIKE && Math.abs(ShooterSubsystem.getHoodVelocity()) <= 10;
        return currentCounter >= 15;
    }

    @Override
    public void end(boolean interrupted){
        climber.zeroClimber();
        climber.hasZeroed = true;
        System.out.println("ZERO DONE");
        //climber.setClimberSpeed(Constants.ClimberHardware.CLIMBER_HOLD_SPEED); // 1.65 is magic number
        climber.gotoClimberPosition(Constants.ClimberHardware.CLIMBER_DOWN_HEIGHT);

    }
}

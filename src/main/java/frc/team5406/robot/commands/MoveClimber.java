package frc.team5406.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team5406.robot.Constants;
import frc.team5406.robot.subsystems.ClimbSubsystem;

public class MoveClimber extends Command{
    
    final ClimbSubsystem climber;
    double desiredPosition;
    double tolerance;

    public MoveClimber(ClimbSubsystem climber, double desiredPosition, double tolerance){
        this.climber = climber;
        this.desiredPosition = desiredPosition;
        this.tolerance = tolerance;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.targetPosition = desiredPosition;
        climber.gotoClimberPosition(desiredPosition);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        if(Math.abs(climber.getClimberPosition() - desiredPosition) < tolerance){
            climber.holdClimberState();
            return true;
        }else{
            return false;
        }
    }
}

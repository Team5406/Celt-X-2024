package frc.team5406.robot.subsystems;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private Spark climberMotor = new Spark(Constants.ClimberHardware.CLIMB_MOTOR_ID, MotorKind.NEO_550);
    static ElevatorFeedforward climbFF = new ElevatorFeedforward(Constants.ClimberHardware.CLIMBER_KS, Constants.ClimberHardware.CLIMBER_KG, Constants.ClimberHardware.CLIMBER_KV);
    
    public boolean hasZeroed = false;
    public double targetPosition = 0;

    public void setupMotors() {
        climberMotor.setSmartCurrentLimit(Constants.ClimberHardware.CURRENT_LIMIT_CLIMBER);
        double climberConversionFactor = (1); // 66/10*88/20
        // spooldiameter 1.5-1.75inch

        climberMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, climberConversionFactor);
        climberMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, climberConversionFactor / 60);

        SparkPIDConfig climberMotorConfig = new SparkPIDConfig(
                Constants.ClimberHardware.CLIMBER_PID,
                Constants.ClimberHardware.CLIMBER_SENSOR_PHASE,
                Constants.ClimberHardware.CLIMBER_INVERT_MOTOR,
                Constants.ClimberHardware.CLIMBER_TOLERANCE,
                Constants.ClimberHardware.CLIMBER_LOWER_LIMIT,
                Constants.ClimberHardware.CLIMBER_UPPER_LIMIT,
                Constants.ClimberHardware.CLIMBER_SOFT_LIMITS);
        // Initialize PID
        climberMotor.initializeSparkPID(climberMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);

        SmartDashboard.putNumber("ClimberKS", Constants.ClimberHardware.CLIMBER_KS);
        SmartDashboard.putNumber("ClimberKV", Constants.ClimberHardware.CLIMBER_KV);
        SmartDashboard.putNumber("ClimberP", Constants.ClimberHardware.CLIMBER_PID.kP);

        zeroClimber();
        climberMotor.burnFlash();
    }

    public void zeroClimber() {
        climberMotor.resetEncoder();
    }

    public double getClimberPosition() {
      return climberMotor.getInputs().encoderPosition;
      }
    
    public double getClimberVelocity() {
      return climberMotor.getInputs().encoderVelocity;
    }
    
    //Change the angle of the arm
    public void stopClimber() {
      gotoClimberPosition(getClimberPosition());
    }
    
    public void setClimberSpeed(double speed) {
      climberMotor.set(speed);
    }

    public void setClimberPositionManual() {
      double position = SmartDashboard.getNumber("Climber Position Target RPM", 0); //FIXME
      gotoClimberPosition(position);
    }
    
    public void gotoClimberPosition(double position) {
      double currentPosition = getClimberPosition();
      double ks = Constants.ClimberHardware.CLIMBER_KS;
      double kg = Constants.ClimberHardware.CLIMBER_KG;
      double kp = Constants.ClimberHardware.CLIMBER_PID.kP;

      double arbFF =  0;
      climberMotor.set(position, ControlType.kPosition, arbFF, SparkPIDController.ArbFFUnits.kVoltage);
    }

    public double getClimberCurrent (){
      return climberMotor.getInputs().current;
    }

    public void holdClimberDown(){
      setClimberSpeed(Constants.ClimberHardware.CLIMBER_HOLD_SPEED);
    }

    public void holdClimberState(){
      gotoClimberPosition(getClimberPosition());
    }

    public void useOutputPosition(double output, TrapezoidProfile.State setpoint) {
      double position = getClimberPosition();
      double arbFF = climbFF.calculate(Units.degreesToRadians(position), Units.degreesToRadians(setpoint.velocity));

      climberMotor.set(setpoint.position, ControlType.kPosition, arbFF, SparkPIDController.ArbFFUnits.kVoltage);
    }

    public ClimbSubsystem() {
      setupMotors();
    }

    public void periodic(){
      climberMotor.periodic();
      SmartDashboard.putNumber("Climber Position", getClimberPosition());
    }
}
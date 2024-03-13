package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

public class TrapSubsystem extends SubsystemBase{
    private Spark trapMotor = new Spark(Constants.TrapHardware.TRAP_MOTOR_ID, MotorKind.NEO_550);
    ArmFeedforward trapFF = new ArmFeedforward(Constants.TrapHardware.TRAP_KS, Constants.TrapHardware.TRAP_KG, Constants.TrapHardware.TRAP_KV);

    public void setupMotors(){
      trapMotor.setSmartCurrentLimit(Constants.TrapHardware.TRAP_CURRENT_LIMIT);
      trapMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  1);
      trapMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  1);

      SparkPIDConfig trapMotorConfig = new SparkPIDConfig(
        Constants.TrapHardware.TRAP_ROTATE_PID,
        Constants.TrapHardware.TRAP_ROTATE_SENSOR_PHASE,
        Constants.TrapHardware.TRAP_ROTATE_INVERT_MOTOR,
        Constants.TrapHardware.TRAP_ROTATE_TOLERANCE
      );
        // Initialize PID
        trapMotor.initializeSparkPID(trapMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);
        trapMotor.setSmartMotionAllowedClosedLoopError(Constants.TrapHardware.TRAP_POSITION_TOLERANCE);
        trapMotor.setIZone(0);
        trapMotor.setSmartMotionMaxVelocity(2000);
        trapMotor.setSmartMotionMaxAccel(5000);
        trapMotor.setMeasurementPeriod();
        trapMotor.setAverageDepth();

        resetTrapAngle();
        trapMotor.burnFlash();
    }

    public double getTrapAngle() {
      return trapMotor.getInputs().encoderPosition;
      }
    
    public double getTrapVelocity() {
      return trapMotor.getInputs().encoderVelocity;
    }
    
    public void resetTrapAngle() {
      trapMotor.resetEncoder(Constants.TrapHardware.TRAP_ZERO_ANGLE);
    }
    
    //Change the angle of the arm
    public void stopTrap() {
      goToTrapAngle(getTrapAngle());
    }
    
    public void setTrapSpeed(double speed) {
      trapMotor.set(speed);
    }

    public void setTrapAngleManual() {
      double angle = SmartDashboard.getNumber("Trap Angle Target RPM", 0); //FIXME
      goToTrapAngle(angle);
    }
    
    public void goToTrapAngle(double angle) {
      trapMotor.set(angle, ControlType.kSmartMotion);
    }

    public double getTrapCurrent(){
      return trapMotor.getInputs().current;
    }

    public TrapSubsystem() {
      setupMotors();
    }


    public void periodic(){
      trapMotor.periodic();
      SmartDashboard.putNumber("Trap Angle", getTrapAngle()); 
    }
}

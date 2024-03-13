package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

public class ArmSubsystem extends SubsystemBase{
    private Spark armMotor = new Spark(Constants.ArmHardware.ARM_MOTOR_ID, MotorKind.NEO);

    double armConversionFactor = (Constants.DEGREES_PER_ROTATION / Constants.ArmHardware.ARM_GEAR_RATIO);
    private boolean climbing = false;

    public void setupMotors(){
      armMotor.setSmartCurrentLimit(Constants.ArmHardware.CURRENT_LIMIT_ARM);

      armMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, 1);
      armMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, 1);

      SparkPIDConfig armMotorConfig = new SparkPIDConfig(
        Constants.ArmHardware.ARM_ROTATE_PID,
        Constants.ArmHardware.ARM_ROTATE_SENSOR_PHASE,
        Constants.ArmHardware.ARM_ROTATE_INVERT_MOTOR,
        Constants.ArmHardware.ARM_ROTATE_TOLERANCE
      );
        // Initialize PID
        armMotor.initializeSparkPID(armMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);
        armMotor.setSmartMotionAllowedClosedLoopError(Constants.ArmHardware.ARM_POSITION_TOLERANCE);
        armMotor.setIZone(0);
        armMotor.setSmartMotionMaxVelocity(5500);
        armMotor.setSmartMotionMaxAccel(8000);
        armMotor.setMeasurementPeriod();
        armMotor.setAverageDepth();

        resetArmAngle();
        armMotor.burnFlash();
    }

    //states of being not actually climbing
    public void climbing(){
      climbing = true;
    }

    public void notClimbing(){
      climbing = false;
    }

    public boolean getClimbing(){
      return climbing;
    }

    public double getArmAngle() {
      return armConversionFactor*armMotor.getInputs().encoderPosition;
      }
    
    public double getArmVelocity() {
      return (armConversionFactor/60) *armMotor.getInputs().encoderVelocity;
    }
    
    public void resetArmAngle() {
      armMotor.resetEncoder(Constants.ArmHardware.ARM_ZERO_ANGLE/armConversionFactor);
    }
    
    //Change the angle of the arm
    public void stopArm() {
      gotoArmAngle(getArmAngle());
    }
    
    public void setArmSpeed(double speed) {
      armMotor.set(speed);
    }

    public void setArmAngleManual() {
      double angle = SmartDashboard.getNumber("Arm Angle Target RPM", 0); //FIXME
      gotoArmAngle(angle);
    }
    
    public void gotoArmAngle(double angle) {
      angle /= armConversionFactor;
      double currentAngle = getArmAngle();
      double arbFF =  Constants.ArmHardware.ARM_KG * Math.cos(Units.degreesToRadians((Constants.ArmHardware.ARM_ZERO_ANGLE+currentAngle)));

      armMotor.set(angle, ControlType.kSmartMotion, arbFF, SparkPIDController.ArbFFUnits.kVoltage);
    }

    public double getArmCurrent (){
      return armMotor.getInputs().current;
    }

    public void useOutputPosition(double output, TrapezoidProfile.State setpoint) {

    }

    public ArmSubsystem() {
      setupMotors();
    }
      
    public void periodic(){
      armMotor.periodic();
      SmartDashboard.putNumber("Arm Angle", getArmAngle()); 
    }
}
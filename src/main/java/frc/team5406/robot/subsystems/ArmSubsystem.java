package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

public class ArmSubsystem extends SubsystemBase{
    private Spark armMotor = new Spark(Constants.ArmHardware.ARM_MOTOR_ID, MotorKind.NEO);

    SimpleMotorFeedforward armFF = new SimpleMotorFeedforward(Constants.ArmHardware.ARM_KS, Constants.ArmHardware.ARM_KV,
    Constants.ArmHardware.ARM_KA);

    public void setupMotors(){
      armMotor.restoreFactoryDefaults();
      armMotor.setSmartCurrentLimit(Constants.ArmHardware.CURRENT_LIMIT_ARM);
      double armConversionFactor = (Constants.DEGREES_PER_ROTATION / Constants.ArmHardware.ARM_GEAR_RATIO);

      armMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  armConversionFactor);
      armMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  armConversionFactor / 60);

      SparkPIDConfig armMotorConfig = new SparkPIDConfig(
        Constants.ArmHardware.ARM_ROTATE_PID,
        Constants.ArmHardware.ARM_ROTATE_SENSOR_PHASE,
        Constants.ArmHardware.ARM_ROTATE_INVERT_MOTOR,
        Constants.ArmHardware.ARM_ROTATE_TOLERANCE,
        Constants.ArmHardware.ARM_ROTATE_LOWER_LIMIT,
        Constants.ArmHardware.ARM_ROTATE_UPPER_LIMIT,
        Constants.ArmHardware.ARM_ROTATE_SOFT_LIMITS
      );
        // Initialize PID
        armMotor.initializeSparkPID(armMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);

        SmartDashboard.putNumber("Arm Angle Target RPM", Constants.ArmHardware.ARM_TARGET_RPM);

        resetArmAngle();
        armMotor.burnFlash();
    }

    public double getArmAngle() {
      return armMotor.getInputs().encoderPosition;
      }
    
    public double getArmVelocity() {
      return armMotor.getInputs().encoderVelocity;
    }
    
    public void resetArmAngle() {
      armMotor.resetEncoder(0);
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
      armMotor.set(angle, ControlType.kPosition);
    }

    public void useOutputPosition(double output, TrapezoidProfile.State setpoint) {

      double angle = getArmAngle();
    
      double arbFF = armFF.calculate(Units.degreesToRadians(angle), Units.degreesToRadians(setpoint.velocity));
      armMotor.set(setpoint.position, ControlType.kPosition, arbFF, SparkPIDController.ArbFFUnits.kVoltage);
      }

      public ArmSubsystem() {
        setupMotors();
      }
      
      public void periodic(){
        armMotor.periodic();
        SmartDashboard.putNumber("Arm Angle", getArmAngle()); 
      }
    }

package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterSubsystem extends SubsystemBase {

    private static Spark shooterMotor = new Spark(Constants.ShooterHardware.SHOOTER_MOTOR_ID, MotorKind.NEO);
    
    SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(Constants.ShooterHardware.SHOOTER_KS, Constants.ShooterHardware.SHOOTER_KV,
    Constants.ShooterHardware.SHOOTER_KA);
    
    public void setupMotors(){
      shooterMotor.restoreFactoryDefaults();
        shooterMotor.setSmartCurrentLimit(Constants.ShooterHardware.SHOOTER_CURRENT_LIMIT);
        double shooterConversionFactor = 1.0;

        shooterMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  shooterConversionFactor);
       
        SparkPIDConfig shooterMotorConfig = new SparkPIDConfig(
          Constants.ShooterHardware.SHOOTER_PID,
          Constants.ShooterHardware.SHOOTER_SENSOR_PHASE,
          Constants.ShooterHardware.SHOOTER_INVERT_MOTOR,
          Constants.ShooterHardware.SHOOTER_TOLERANCE
        );
         // Initialize PID
        shooterMotor.initializeSparkPID(shooterMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);
        shooterMotor.burnFlash();
      
        SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterHardware.SHOOTER_TARGET_RPM);
    }

    public  void setShooterSpeed(double RPM) {
        if (RPM == 0) {
          shooterMotor.set(0);
        } else {
   double arbFF = shooterFF.calculate(RPM / 60.0);
        shooterMotor.set(RPM, ControlType.kVelocity, arbFF,
           SparkPIDController.ArbFFUnits.kVoltage);
        }
      }

    public  void setShooterManual() {
        double RPM = SmartDashboard.getNumber("Shooter Target RPM", Constants.ShooterHardware.SHOOTER_TARGET_RPM);
        setShooterSpeed(RPM);
    }

    public void set(double speed) {
      shooterMotor.set(speed);
  }

    public  double getShooterSpeed() {
        return shooterMotor.getInputs().encoderVelocity;
    }

    public void stopShooter(){
      setShooterSpeed(0);
    }

    public ShooterSubsystem() {
      setupMotors();
    }

  public void periodic() {
    shooterMotor.periodic();
    SmartDashboard.putNumber("Shooter Speed", getShooterSpeed()); 
  }
}

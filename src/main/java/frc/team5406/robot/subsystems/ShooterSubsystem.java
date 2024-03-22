package frc.team5406.robot.subsystems;

import frc.team5406.robot.Constants;
import frc.team5406.robot.Constants.ShooterHardware;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterSubsystem extends SubsystemBase {

  private Spark shooterMotorLeftOne = new Spark(Constants.ShooterHardware.SHOOTER_MOTOR_ID_LEFT_ONE, MotorKind.NEO_VORTEX);
  private Spark shooterMotorRightOne = new Spark(Constants.ShooterHardware.SHOOTER_MOTOR_ID_RIGHT_ONE, MotorKind.NEO_VORTEX);

  SimpleMotorFeedforward shooterFFLeft = new SimpleMotorFeedforward(Constants.ShooterHardware.SHOOTER_KS_LEFT, Constants.ShooterHardware.SHOOTER_KV_LEFT,
    Constants.ShooterHardware.SHOOTER_KA_LEFT);
  SimpleMotorFeedforward shooterFFRight = new SimpleMotorFeedforward(Constants.ShooterHardware.SHOOTER_KS_RIGHT, Constants.ShooterHardware.SHOOTER_KV_RIGHT,
    Constants.ShooterHardware.SHOOTER_KA_RIGHT);

  public void setupMotors(){

    shooterMotorLeftOne.setSmartCurrentLimit(Constants.ShooterHardware.SHOOTER_CURRENT_LIMIT);
    shooterMotorRightOne.setSmartCurrentLimit(Constants.ShooterHardware.SHOOTER_CURRENT_LIMIT);

    double shooterConversionFactor = 1.0;

    shooterMotorLeftOne.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, shooterConversionFactor);
    shooterMotorRightOne.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, shooterConversionFactor);
  
    SparkPIDConfig shooterMotorConfigLeft = new SparkPIDConfig(
      Constants.ShooterHardware.SHOOTER_PID,
      Constants.ShooterHardware.SHOOTER_SENSOR_PHASE,
      Constants.ShooterHardware.SHOOTER_INVERT_MOTOR_LEFT,
      Constants.ShooterHardware.SHOOTER_TOLERANCE
    );

    SparkPIDConfig shooterMotorConfigRight = new SparkPIDConfig(
      Constants.ShooterHardware.SHOOTER_PID,
      Constants.ShooterHardware.SHOOTER_SENSOR_PHASE,
      Constants.ShooterHardware.SHOOTER_INVERT_MOTOR_RIGHT,
      Constants.ShooterHardware.SHOOTER_TOLERANCE
    );
      
    // Initialize PID
    shooterMotorLeftOne.initializeSparkPID(shooterMotorConfigLeft, Spark.FeedbackSensor.NEO_ENCODER);
    shooterMotorRightOne.initializeSparkPID(shooterMotorConfigRight, Spark.FeedbackSensor.NEO_ENCODER);

    shooterMotorLeftOne.setMeasurementPeriod();
    shooterMotorLeftOne.setAverageDepth();
    
    shooterMotorRightOne.setMeasurementPeriod();
    shooterMotorRightOne.setAverageDepth();

    shooterMotorLeftOne.burnFlash();
    shooterMotorRightOne.burnFlash();

    SmartDashboard.putNumber("Shooter Target RPM", Constants.ShooterHardware.SHOOTER_TARGET_RPM);

  }

  public void setShooterSpeed(double RPM, boolean same) {
    if (RPM == 0) {
      shooterMotorRightOne.set(0);
      shooterMotorLeftOne.set(0);
    } else {
      double arbFFLeft = shooterFFLeft.calculate(RPM / 60.0);
      shooterMotorLeftOne.set(RPM, ControlType.kVelocity, arbFFLeft, SparkPIDController.ArbFFUnits.kVoltage);
      if(same){
      double arbFFRight = shooterFFRight.calculate(RPM / 60.0);
      shooterMotorRightOne.set(RPM, ControlType.kVelocity, arbFFRight, SparkPIDController.ArbFFUnits.kVoltage);
      }else{
      double arbFFRight = shooterFFRight.calculate(ShooterHardware.SHOOTER_SPEED_MULTIPLIER * RPM / 60.0);
      shooterMotorRightOne.set(ShooterHardware.SHOOTER_SPEED_MULTIPLIER * RPM, ControlType.kVelocity, arbFFRight, SparkPIDController.ArbFFUnits.kVoltage);
      }
    }

  }

    public void setShooterSpeeds(double left, double right) {
      double arbFFLeft = shooterFFLeft.calculate(left / 60.0);
      shooterMotorLeftOne.set(left, ControlType.kVelocity, arbFFLeft, SparkPIDController.ArbFFUnits.kVoltage);
      double arbFFRight = shooterFFRight.calculate(right / 60.0);
      shooterMotorRightOne.set(right, ControlType.kVelocity, arbFFRight, SparkPIDController.ArbFFUnits.kVoltage);
    

  }

  public void setShooterSpeed(double RPM) {
      setShooterSpeed(RPM, false);
    }

  public void setShooterManual() {
    double RPM = SmartDashboard.getNumber("Shooter Target RPM", Constants.ShooterHardware.SHOOTER_TARGET_RPM);
    setShooterSpeed(RPM);
  }

  public void set(double speed) {
    shooterMotorLeftOne.set(speed);
    shooterMotorRightOne.set(speed * ShooterHardware.SHOOTER_SPEED_MULTIPLIER);
  }

  public double getShooterSpeedLeft() {
    return shooterMotorLeftOne.getInputs().encoderVelocity;
  }
      
  public double getShooterSpeedRight() {
    return shooterMotorRightOne.getInputs().encoderVelocity;
  }

  public boolean getShouldRumble(){
    
    return getShooterSpeedLeft() > 3300;
  }

  public void stopShooter(){
    setShooterSpeed(0);
  }

  public ShooterSubsystem() {
    setupMotors();
  }

  public void periodic() {
    shooterMotorLeftOne.periodic();
    shooterMotorRightOne.periodic();

    SmartDashboard.putNumber("Shooter Speed Left", getShooterSpeedLeft()); 
    SmartDashboard.putNumber("Shooter Speed Right", getShooterSpeedRight());
    SmartDashboard.putBoolean("Shooter Speed Good?", getShooterSpeedLeft() > 3300);
  }
}

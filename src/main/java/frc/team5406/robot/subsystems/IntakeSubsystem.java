package frc.team5406.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;


import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;


public class IntakeSubsystem extends SubsystemBase {

  private Spark intakeMotor = new Spark(Constants.IntakeHardware.INTAKE_MOTOR_ID, MotorKind.NEO_VORTEX);
  SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(Constants.IntakeHardware.INTAKE_KS, Constants.IntakeHardware.INTAKE_KV,
  Constants.IntakeHardware.INTAKE_KA);
  
  public void setupMotors(){
    intakeMotor.setSmartCurrentLimit(Constants.IntakeHardware.INTAKE_CURRENT_LIMIT);

    double intakeConversionFactor = 1.0;

    intakeMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER, intakeConversionFactor);

    SparkPIDConfig intakeMotorConfig = new SparkPIDConfig(
      Constants.IntakeHardware.INTAKE_PID,
      Constants.IntakeHardware.INTAKE_SENSOR_PHASE,
      Constants.IntakeHardware.INTAKE_INVERT_MOTOR,
      Constants.IntakeHardware.INTAKE_TOLERANCE
    );

    intakeMotor.initializeSparkPID(intakeMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);
    intakeMotor.burnFlash();
  
    SmartDashboard.putNumber("Intake Target RPM", Constants.IntakeHardware.INTAKE_TARGET_RPM);
  }

  public void setIntakeSpeed(double RPM) {
    if (RPM == 0) {
      intakeMotor.set(0);
    } else {
      double arbFF = intakeFF.calculate(RPM);
      intakeMotor.set(RPM,ControlType.kVelocity, arbFF, SparkPIDController.ArbFFUnits.kVoltage);
    }

  }


  public void setIntakeManual() {
    double RPM = SmartDashboard.getNumber("Intake Target RPM", Constants.IntakeHardware.INTAKE_TARGET_RPM);
    setIntakeSpeed(RPM);
  }
  
  public void set(double speed) {
    intakeMotor.set(speed);
  }

  public double getIntakeSpeed() {
    return intakeMotor.getInputs().encoderVelocity;
  }

  public void stopIntake(){
    setIntakeSpeed(0);
  }

  public IntakeSubsystem() {
    setupMotors();
  }

  public void periodic() {
    intakeMotor.periodic();
    SmartDashboard.putNumber("Intake Speed",getIntakeSpeed());
  }
}
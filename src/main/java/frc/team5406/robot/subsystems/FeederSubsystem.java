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


public class FeederSubsystem extends SubsystemBase {

    private Spark feederMotor = new Spark(Constants.FeederHardware.FEEDER_MOTOR_ID, MotorKind.NEO);
    
    SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(Constants.FeederHardware.FEEDER_KS, Constants.FeederHardware.FEEDER_KV,
    Constants.FeederHardware.FEEDER_KA);
    
    public void setupMotors(){

    feederMotor.setSmartCurrentLimit(Constants.FeederHardware.FEEDER_CURRENT_LIMIT);
    feederMotor.restoreFactoryDefaults();
    double feederConversionFactor = 1.0;

 feederMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  feederConversionFactor);
    SparkPIDConfig feederMotorConfig = new SparkPIDConfig(
        Constants.FeederHardware.FEEDER_PID,
        Constants.FeederHardware.FEEDER_SENSOR_PHASE,
        Constants.FeederHardware.FEEDER_INVERT_MOTOR,
        Constants.FeederHardware.FEEDER_TOLERANCE
    );
        feederMotor.burnFlash();       
        feederMotor.initializeSparkPID(feederMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);

        SmartDashboard.putNumber("Feeder Target RPM", Constants.FeederHardware.FEEDER_TARGET_RPM);
    }

    public  void setFeederSpeed(double RPM) {
        if (RPM == 0) {
          feederMotor.set(0);
        } else {
   double arbFF = feederFF.calculate(RPM / 60);
        feederMotor.set(RPM, ControlType.kVelocity, arbFF,
           SparkPIDController.ArbFFUnits.kVoltage);
        }
      }

    public  void setFeederManual() {
        double RPM = SmartDashboard.getNumber("Feeder Target RPM", Constants.FeederHardware.FEEDER_TARGET_RPM); 
        setFeederSpeed(RPM);
    }

    public  void set(double speed) {
    feederMotor.set(speed);   
   }

   public double getFeederSpeed() {
      return feederMotor.getInputs().encoderVelocity;
   }

    public void stopFeeder(){
      setFeederSpeed(0);

    }

    public FeederSubsystem() {
      setupMotors();
    }
 
  public void periodic() {
    feederMotor.periodic();
    SmartDashboard.putNumber("Feeder Speed", getFeederSpeed());
  }
}

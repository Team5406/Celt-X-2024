package frc.team5406.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import frc.team5406.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Distance;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.apache.commons.math3.optimization.Target;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.hardware.PWF.ToFSensor;


import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.playingwithfusion.TimeOfFlight.Status;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class FeederSubsystem extends SubsystemBase {
  private final ToFSensor m_rangeSensor = new ToFSensor(Constants.FeederHardware.TOF_SENSOR_ID);

  private Spark feederMotor = new Spark(Constants.FeederHardware.FEEDER_MOTOR_ID, MotorKind.NEO);
  
  SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(Constants.FeederHardware.FEEDER_KS, Constants.FeederHardware.FEEDER_KV,
  Constants.FeederHardware.FEEDER_KA);
  
  double noteTime = 0;
  boolean doneBefore = false;
  int invalidCount = 0;
  boolean lastNote = false;

  public void setupMotors(){

    feederMotor.setSmartCurrentLimit(Constants.FeederHardware.FEEDER_CURRENT_LIMIT);

    double feederConversionFactor = 1.0;

    feederMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  feederConversionFactor);

    SparkPIDConfig feederMotorConfig = new SparkPIDConfig(
      Constants.FeederHardware.FEEDER_PID,
      Constants.FeederHardware.FEEDER_SENSOR_PHASE,
      Constants.FeederHardware.FEEDER_INVERT_MOTOR,
      Constants.FeederHardware.FEEDER_TOLERANCE
    );

    feederMotor.initializeSparkPID(feederMotorConfig, Spark.FeedbackSensor.NEO_ENCODER);
    m_rangeSensor.setRangingMode(RangingMode.Short, Constants.FeederHardware.TOF_SAMPLE_TIME);
    
    feederMotor.burnFlash();       
    SmartDashboard.putNumber("Feeder Target RPM", Constants.FeederHardware.FEEDER_TARGET_RPM);

  }

  public void setFeederSpeed(double RPM) {
    if (RPM == 0) {
      feederMotor.set(0);
    } else {
      double arbFF = feederFF.calculate(RPM / 60);
      feederMotor.set(RPM, ControlType.kVelocity, arbFF,
      SparkPIDController.ArbFFUnits.kVoltage);
    }
  }

  public void setFeederManual() {
    double RPM = SmartDashboard.getNumber("Feeder Target RPM", Constants.FeederHardware.FEEDER_TARGET_RPM); 
    setFeederSpeed(RPM);
  }

  public void set(double speed) {
    feederMotor.set(speed);   
  }

  public void gotoFeederPosition(double position) {
    feederMotor.set(position, ControlType.kPosition);
  }

  public void resetPosition(){
    feederMotor.resetEncoder(0);
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
    if (getToFStatus()){
      SmartDashboard.putNumber("Feeding position", getToFDistance());
    }
    SmartDashboard.putBoolean("FeederStatus", getToFStatus());
    
  }

  public double getToFDistance() {
    return m_rangeSensor.getToFDistance(Constants.FeederHardware.TOF_OFFSET); 
  }

  public boolean getToFStatus() {
    return m_rangeSensor.isValid();
  }
  
    public boolean haveNote() {
      if(getToFStatus()){
        invalidCount=0;
        lastNote = (getToFDistance() - Constants.FeederHardware.TOF_NOTE_SEEN_THRESHOLD) < Constants.FeederHardware.FEEDER_SETPOINT_TOLERANCE;
        if(lastNote && !doneBefore){
          doneBefore = true;
          noteTime = Timer.getFPGATimestamp();
        }else if(!lastNote){
          doneBefore = false;
        }
        return lastNote;
      }else if(invalidCount < 50){
        invalidCount++;
        return lastNote;
      }else{
        doneBefore = false;
        return false;
      }
    }



  public boolean getShouldRumble(){
    SmartDashboard.putNumber("Elapsed Note Time", Timer.getFPGATimestamp() - noteTime);
    SmartDashboard.putBoolean("Have Note", haveNote());
    SmartDashboard.putNumber("Note Time", noteTime);

    return haveNote() && (Timer.getFPGATimestamp() - noteTime)  < Constants.FeederHardware.RUMBLE_TIME_LIMIT;
  }

  


}
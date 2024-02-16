// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot.subsystems.shooter;

import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.FeedbackSensor;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.hardware.revrobotics.SparkPIDConfig;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team5406.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public static class Hardware {
    Spark armMotor;
    Spark indexerMotor;
    Spark flywheelMotor;

    public Hardware(Spark armMotor, Spark indexerMotor, Spark flywheelMotor) {
      this.armMotor = armMotor;
      this.indexerMotor = indexerMotor;
      this.flywheelMotor = flywheelMotor;
    }
  }

  /** Shooter state */
  public static class State {
    public final Measure<Angle> armAngle;
    public final Measure<Velocity<Angle>> flywheelSpeed;

    // List of shooter states
    // Default should probably be your resting position
    public static final State DEFAULT = new State(Units.Degrees.of(0.0), Units.RPM.of(0.0)); // TODO: Change this to the actual default position
    public static final State INTAKE_SOURCE = new State(Units.Degrees.of(-15.0), Units.RPM.of(-4000));
    public static final State SHOOT_PODIUM = new State(Units.Degrees.of(-18.0), Units.RPM.of(+6000));
    public static final State SHOOT_PODIUM_BACKWARDS = new State(Units.Degrees.of(-75.0), Units.RPM.of(+6000));
    public static final State SHOOT_SUBWOOFER = new State(Units.Degrees.of(-27.0), Units.RPM.of(+5000));
    public static final State SHOOT_SUBWOOFER_BACKWARDS = new State(Units.Degrees.of(-75.0), Units.RPM.of(+5000));

    public State(Measure<Angle> armAngle, Measure<Velocity<Angle>> flywheelSpeed) {
      this.armAngle = armAngle;
      this.flywheelSpeed = flywheelSpeed;
    }
  }

  // I like to set things that aren't commonly changed and things that are inherent to the mechanisms in the subsystem itself.
  // and use the Constants.java file for things that are more adjustable "settings". Of course, this is a preference.
  private static final Measure<Current> ARM_MOTOR_CURRENT_LIMIT = Units.Amps.of(40.0);
  private static final Measure<Current> INDEXER_MOTOR_CURRENT_LIMIT = Units.Amps.of(40.0);
  private static final Measure<Current> FLYWHEEL_MOTOR_CURRENT_LIMIT = Units.Amps.of(70.0);
  private static final Measure<Velocity<Angle>> INDEXER_SPEED = Units.RPM.of(5000);
  private static final Measure<Velocity<Angle>> INDEXER_SLOW_SPEED = Units.RPM.of(1000);

  private static final double ARM_GEAR_RATIO = 64.0;

  private Spark m_armMotor;
  private Spark m_indexerMotor;
  private Spark m_flywheelMotor;

  private SparkPIDConfig m_armConfig;
  private SparkPIDConfig m_indexerConfig;
  private SparkPIDConfig m_flywheelConfig;

  private State m_desiredShooterState;
  private SimpleMotorFeedforward m_armFF;
  private TrapezoidProfile.Constraints m_armMotionConstraints;

  /**
   * Create an instance of ShooterSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param shooterHardware Hardware devices required by shooter
   * @param armConfig Arm PID config
   * @param indexerConfig Indexer PID config
   * @param flywheelConfig Flywheel PID config
   * @param armFF Arm feed forward
   * @param armMotionConstraints Arm motion constraint
   */
  public ShooterSubsystem(Hardware shooterHardware, SparkPIDConfig armConfig, SparkPIDConfig indexerConfig, SparkPIDConfig flywheelConfig,
                          SimpleMotorFeedforward armFF, TrapezoidProfile.Constraints armMotionConstraints) {
    this.m_armMotor = shooterHardware.armMotor;
    this.m_indexerMotor = shooterHardware.indexerMotor;
    this.m_flywheelMotor = shooterHardware.flywheelMotor;
    this.m_armConfig = armConfig;
    this.m_indexerConfig = indexerConfig;
    this.m_flywheelConfig = flywheelConfig;
    this.m_armFF = armFF;
    this.m_armMotionConstraints = armMotionConstraints;

    // Set arm conversion factors
    var armConversionFactor = (Constants.DEGREES_PER_ROTATION / ARM_GEAR_RATIO);
    m_armMotor.setPositionConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  armConversionFactor);
    m_armMotor.setVelocityConversionFactor(Spark.FeedbackSensor.NEO_ENCODER,  armConversionFactor / 60);

    // Initialize arm PID
    m_armMotor.initializeSparkPID(m_armConfig, FeedbackSensor.NEO_ENCODER);

    // Initialize indexer PID
    m_indexerMotor.initializeSparkPID(m_indexerConfig, FeedbackSensor.NEO_ENCODER);

    // Initialize flywheel PID
    m_flywheelMotor.initializeSparkPID(m_flywheelConfig, FeedbackSensor.NEO_ENCODER);

    // Set current limits
    m_armMotor.setSmartCurrentLimit((int)ARM_MOTOR_CURRENT_LIMIT.in(Units.Amps));
    m_indexerMotor.setSmartCurrentLimit((int)INDEXER_MOTOR_CURRENT_LIMIT.in(Units.Amps));
    m_flywheelMotor.setSmartCurrentLimit((int)FLYWHEEL_MOTOR_CURRENT_LIMIT.in(Units.Amps));

    // Initialize current state
    m_desiredShooterState = getCurrentState();
  }

  /**
   * Initialize hardware devices for shooter subsystem
   * @return Hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware shooterHardware = new Hardware(
      new Spark(Constants.ArmHardware.ARM_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.FeederHardware.FEEDER_MOTOR_ID, MotorKind.NEO),
      new Spark(Constants.ShooterHardware.SHOOTER_MOTOR_ID, MotorKind.NEO)
    );

    return shooterHardware;
  }

  /**
   * Set shooter state
   * @param state Desired state
   */
  private void setState(State state) {
    m_desiredShooterState = state;
    m_armMotor.smoothMotion(state.armAngle.in(Units.Degrees), m_armMotionConstraints, this::armFFCalculator);
    m_flywheelMotor.set(state.flywheelSpeed.in(Units.RPM), ControlType.kVelocity);
  }

  /**
   * Get current shooter state
   * @return Current shooter state
   */
  private State getCurrentState() {
    return new State(
      Units.Degrees.of(m_armMotor.getInputs().encoderPosition),
      Units.RPM.of(m_flywheelMotor.getInputs().encoderVelocity)
    );
  }

  /**
   * Set shooter to default state
   */
  private void resetState() {
    setState(State.DEFAULT);
  }

  /**
   * Feed forward calculator for shooter angle
   * @param state Current motion profile state
   * @return Feed forward voltage to apply
   */
  private double armFFCalculator(TrapezoidProfile.State state) {
    return m_armFF.calculate(state.velocity);
  }


  private boolean isReady() {
    return m_armMotor.isSmoothMotionFinished() &&
      Math.abs(m_flywheelMotor.getInputs().encoderVelocity - m_desiredShooterState.flywheelSpeed.in(Units.RPM)) < m_flywheelConfig.getTolerance();
  }

  /**
   * Run indexer to feed note into flywheel
   */
  private void indexerFeed() {
    m_indexerMotor.set(+INDEXER_SPEED.in(Units.RPM), ControlType.kVelocity);
  }

  /**
   * Run indexer to accept note
   */
  private void indexerIntake() {
    m_indexerMotor.set(-INDEXER_SLOW_SPEED.in(Units.RPM), ControlType.kVelocity);
  }

  /**
   * Stop indexer
   */
  private void indexerStop() {
    m_indexerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_armMotor.periodic();
    m_indexerMotor.periodic();
    m_flywheelMotor.periodic();
  }

  /**
   * Intake a game piece from the source
   * @return Command to intake game piece
   */
  public Command intakeCommand() {
    return startEnd(
      () -> { // Run this on command start (i.e. When the button is pressed)
        setState(State.INTAKE_SOURCE);
        indexerIntake();
      },
      () -> { // Run this on command end (i.e. When the button is released)
        resetState();
        indexerStop();
      }
    );
  }

  /**
   * Shoot by manually specifying a desired shooter state
   * @param state Desired state
   * @return Command that will shoot game piece with desired parameters
   */
  public Command shootManualCommand(State state) {
    return runEnd(
      () -> { // Run this every loop while command is scheduled (i.e. while button is held)
        if (isReady()) indexerFeed();
        else indexerStop();
      },
      () -> { // Run this on command end (i.e. When the button is released)
        resetState();
        indexerStop();
      }
    ).beforeStarting(() -> setState(state), this); // Before starting the 'runEnd' above, do this
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import java.util.Optional;
import java.util.function.DoubleConsumer;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.SDSMK4SwerveModule;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.hardware.ctre.PhoenixCANBus;
import org.lasarobotics.hardware.PWF.ToFSensor;
import org.lasarobotics.hardware.ctre.CANCoder;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team5406.robot.subsystems.vision.AprilTagCamera.Resolution;
import frc.team5406.robot.subsystems.vision.VisionSubsystem;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int DEGREES_PER_ROTATION = 360;
  public static final double TAG_TURN_P = 0;
  public static final double TAG_TURN_I = 0;
  public static final double TAG_TURN_D = 0;
  public static final double TAG_TURN_TOLERANCE = 0;

  public static final AprilTag BLUE_SPEAKER_TAG = VisionSubsystem.getInstance().getTag(7).get();
  public static final AprilTag RED_SPEAKER_TAG = VisionSubsystem.getInstance().getTag(4).get();

  public static class HID {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.10;
    public static final int OPERATOR_CONTROLLER_PORT = 0;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_TURN_PID = new PIDConstants(8.0, 0.0, 0.3, 0.0,0.0);
    public static final double DRIVE_SLIP_RATIO = 0.08;
    public static final double DRIVE_TURN_SCALAR = 30.0;
    public static final double DRIVE_LOOKAHEAD = 3;

    public static final ControlCentricity DRIVE_CONTROL_CENTRICITY = ControlCentricity.FIELD_CENTRIC;

    private static final double DRIVE_THROTTLE_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.000 };
    private static final double DRIVE_THROTTLE_INPUT_CURVE_Y[] = { 0.0, 0.042, 0.168, 0.378, 0.672, 1.050, 1.512, 2.508, 2.688, 3.402, 4.300 };
    private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
    private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.008, 0.032, 0.072, 0.128, 0.200, 0.288, 0.392, 0.512, 0.768, 1.0 };

    private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final PolynomialSplineFunction DRIVE_THROTTLE_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_THROTTLE_INPUT_CURVE_X, DRIVE_THROTTLE_INPUT_CURVE_Y);
    public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);

    public static final SDSMK4SwerveModule.GearRatio DRIVE_GEAR_RATIO = SDSMK4SwerveModule.GearRatio.L2;
    public static final double DRIVE_WIDTH = Units.Inches.of(17.32).in(Units.Meters);
    public static final double DRIVE_LENGTH = Units.Inches.of(21.5).in(Units.Meters);
  }

  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 1);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 2);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 3);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 4);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 5);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 6);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 7);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 8);
    public static final CANCoder.ID LEFT_FRONT_ENCODER = new CANCoder.ID("DriveHardware/Swerve/LeftFront/Encoder", PhoenixCANBus.RIO, 32);
    public static final CANCoder.ID LEFT_REAR_ENCODER = new CANCoder.ID("DriveHardware/Swerve/LeftRear/Encoder", PhoenixCANBus.RIO, 31);
    public static final CANCoder.ID RIGHT_FRONT_ENCODER = new CANCoder.ID("DriveHardware/Swerve/RightFront/Encoder", PhoenixCANBus.RIO, 33);
    public static final CANCoder.ID RIGHT_REAR_ENCODER = new CANCoder.ID("DriveHardware/Swerve/RightRear/Encoder", PhoenixCANBus.RIO, 34);
  }

  public static class ArmHardware {
    public static final Spark.ID ARM_MOTOR_ID = new Spark.ID("ArmHardware", 11);
    public static final double ARM_GEAR_RATIO = 17.0*66.0/12.0;
    public static final Measure<Current> CURRENT_LIMIT_ARM = Units.Amps.of(40);  
    public static final PIDConstants ARM_ROTATE_PID = new PIDConstants( 0.00007, 0.000001, 0.000001, 0.000156, 0.0);
    public static final double ARM_TARGET_RPM = 0;
    public static final boolean ARM_ROTATE_SENSOR_PHASE=false;
    public static final boolean ARM_ROTATE_INVERT_MOTOR=true;
    public static final double ARM_ROTATE_TOLERANCE=0.1;
    public static final double ARM_ROTATE_LOWER_LIMIT=0;
    public static final double ARM_ROTATE_UPPER_LIMIT=0;
    public static final boolean ARM_ROTATE_SOFT_LIMITS=false;
    public static final double ARM_KS = 0;
    public static final double ARM_KV = 0.05;
    public static final double ARM_KA = 0.1;
    public static final double ARM_KG = 1.0; //FIXME
    public static final double ARM_PID_PROFILED_P = 0.4;
    public static final double ARM_PID_PROFILED_I = 0;
    public static final double ARM_PID_PROFILED_D = 0;
    public static final double ARM_MAX_SPEED = 225;
    public static final double ARM_MAX_ACCELERATION = 450;
    public static final double ARM_POSITION_TOLERANCE = 0.1;
    public static final double ARM_ZERO_ANGLE = 15;
    public static final double ARM_ZEROING_SPEED = -0.165;
    public static final double CURRENT_SPIKE = 40;
  }

public static class FeederHardware {
    public static final Spark.ID FEEDER_MOTOR_ID = new Spark.ID("FeederHardware", 13);
    public static final ToFSensor.ID TOF_SENSOR_ID = new ToFSensor.ID("FeederToFSensor", 1);
    public static final Measure<Current> FEEDER_CURRENT_LIMIT = Units.Amps.of(40);
    public static final double FEEDER_GEAR_RATIO = 11.0/34.0;
    public static final double FEEDER_TARGET_RPM = 60;
    public static final boolean FEEDER_SENSOR_PHASE=true;
    public static final boolean FEEDER_INVERT_MOTOR=true;
    public static final double FEEDER_TOLERANCE=0.1;
    public static final PIDConstants FEEDER_PID = new PIDConstants( 5.0042E-05, 0.0, 0.0, 0.0, 0.0);
    public static final double FEEDER_KS = 0.23627;//0.19095;
    public static final double FEEDER_KV = 0.38699;
    public static final double FEEDER_KA = 0.03123;
    public static final double TOF_OFFSET = 0.0;
    public static final double TOF_SAMPLE_TIME = 40.0;
    public static final double FEEDER_SPIN_P = 2;
    public static final double FEEDER_SPIN_I = 0.0;
    public static final double FEEDER_SPIN_D = 0.0;
    public static final double FEEDER_NOTE_SETPOINT = 95;
    public static final double FEEDER_SETPOINT_TOLERANCE = 10;
    public static final double TOF_NOTE_SEEN_THRESHOLD = 150;
    public static final double TOF_NOTE_PASSED_THRESHOLD = 350;
    public static final double RUMBLE_TIME_LIMIT = 2;
    public static final double OUTTAKE_FEEDER_SPEED = 300;
}

  public static class ShooterHardware {
    public static final double SHOOTER_TARGET_RPM = 3000;
    public static final Measure<Current> SHOOTER_CURRENT_LIMIT = Units.Amps.of(60);
    public static final double ARM_GEAR_RATIO = 36.0/28.0; //FIXME
    public static final PIDConstants SHOOTER_PID = new PIDConstants( 0.000122, 0.0, 0.0, 0.0, 0.0);
    public static final boolean SHOOTER_SENSOR_PHASE=false;
    public static final boolean SHOOTER_INVERT_MOTOR_RIGHT=true;
    public static final boolean SHOOTER_INVERT_MOTOR_LEFT=false;
    public static final double SHOOTER_TOLERANCE=0.1;
    public static final boolean SHOOTER_SOFT_LIMITS=false;
    public static final double SHOOTER_MULTIPLIER = 6784;
    public static final double SHOOTER_KS_LEFT = 0.15; 
    public static final double SHOOTER_KV_LEFT = 0.11;
    public static final double SHOOTER_KA_LEFT = 0.0750;
    public static final double SHOOTER_KS_RIGHT = 0.15;
    public static final double SHOOTER_KV_RIGHT = 0.11;
    public static final double SHOOTER_KA_RIGHT = 0.0821;
    public static final double SHOOTER_OUTTAKE_SPEED = -500;

    public static final Spark.ID SHOOTER_MOTOR_ID_LEFT_ONE = new Spark.ID("ShooterHardware", 14);
    public static final Spark.ID SHOOTER_MOTOR_ID_RIGHT_ONE = new Spark.ID("ShooterHardware", 16);
    public static final double SHOOTER_SPEED_MULTIPLIER = 0.85;
    public static final int TOF_NOTE_SEEN_THRESHOLD = 0;
    public static final double SPEED_THRESHOLD = 50;
  }

  public static class SourceIntakeHardware {
    public static final double INTAKE_SET_ANGLE = -15;
    public static final double INTAKE_SET_SPEED = -4000;
    public static final double INTAKE_FEEDER_SPEED = -1000;
  }

  public static class IntakeHardware {
    public static final Spark.ID INTAKE_MOTOR_ID = new Spark.ID("IntakeHardware", 9);
    public static final Measure<Current> INTAKE_CURRENT_LIMIT = Units.Amps.of(60);
    public static final double INTAKE_GEAR_RATIO = 18.0/36.0; //FIXME
    public static final double INTAKE_TARGET_RPM = 1500;
    public static final boolean INTAKE_SENSOR_PHASE=false;
    public static final boolean INTAKE_INVERT_MOTOR=true;
    public static final double INTAKE_TOLERANCE=0.1;
    public static final PIDConstants INTAKE_PID = new PIDConstants( 8.09E-03, 0.0, 0.0, 0.0, 0.0);
    public static final double INTAKE_KS = 0.237;//0.19095;
    public static final double INTAKE_KV = 0.129;
    public static final double INTAKE_KA = 0.0293;
    public static final double OUTTAKE_TARGET_RPM = -500;
  }

  public static class TrapHardware{
    public static final Spark.ID TRAP_MOTOR_ID = new Spark.ID("TrapHardware", 18);
    public static final Measure<Current> TRAP_CURRENT_LIMIT = Units.Amps.of(20);
    public static final double TRAP_GEAR_RATIO = 48/1;
    public static final PIDConstants TRAP_ROTATE_PID = new PIDConstants(0.00005, 0.0, 0, 0.000156, 0.0);
    public static final double TRAP_TARGET_RPM = 0;
    public static final boolean TRAP_ROTATE_SENSOR_PHASE=false;
    public static final boolean TRAP_ROTATE_INVERT_MOTOR=true;
    public static final double TRAP_ROTATE_TOLERANCE= 2;
    public static final double TRAP_ROTATE_LOWER_LIMIT=0;
    public static final double TRAP_ROTATE_UPPER_LIMIT=0;
    public static final boolean TRAP_ROTATE_SOFT_LIMITS=false;
    public static final double TRAP_KS = 0;
    public static final double TRAP_KV = 0.0;
    public static final double TRAP_KA = 0.0;
    public static final double TRAP_KG = 0.0;
    public static final double TRAP_PID_PROFILED_P = 0.0;
    public static final double TRAP_PID_PROFILED_I = 0;
    public static final double TRAP_PID_PROFILED_D = 0;
    public static final double TRAP_MAX_SPEED = 100;
    public static final double TRAP_MAX_ACCELERATION = 100;
    public static final double TRAP_POSITION_TOLERANCE = 0.1;
    public static final double TRAP_ZEROING_SPEED = -0.165; //FIXME
    public static final double TRAP_ZERO_ANGLE = 0;
    public static final double TRAP_MAX_ANGLE = 158;
    public static final double CURRENT_SPIKE = 20;
  }

  public static class ClimberHardware {
    public static final Spark.ID CLIMB_MOTOR_ID = new Spark.ID("ClimbHardware", 19);
    public static final double CLIMBER_KS = 1; //FIXME
    public static final double CLIMBER_KV = 1; //FIXME
    public static final double CLIMBER_KA = 1; //FIXME
    public static final double CLIMBER_KG = 2; //FIXME

    public static final Measure<Current> CURRENT_LIMIT_CLIMBER = Units.Amps.of(50); //FIXME
    public static final double CLIMBER_GEAR_RATIO = (66/10)*(88/20);
    public static final PIDConstants CLIMBER_PID = new PIDConstants(0.04, 0, 0, 0, 0.0); //FIXME
    public static final boolean CLIMBER_SENSOR_PHASE = false;
    public static final boolean CLIMBER_INVERT_MOTOR = true;
    public static final double CLIMBER_TOLERANCE = 0.1; //FIXME
    public static final double CLIMBER_LOWER_LIMIT = -1;
    public static final double CLIMBER_UPPER_LIMIT = 1;
    public static final boolean CLIMBER_SOFT_LIMITS = false;
    public static final double spoolDiameter = Units.Inches.of(1.75).in(Units.Meters);
    
    public static final double CLIMBER_ZEROING_SPEED = -0.5; //FIXME
    public static final int CURRENT_SPIKE = 50; //FIXME
    public static final double CLIMBER_HOLD_SPEED = -0.025; //FIXME
    public static final double CLIMBER_HEIGHT_THRESHOLD = 1.5; //FIXME
    public static final double CLIMBER_MAX_HEIGHT = 160; //FIXME
    public static final double CLIMBER_VELOCITY_THRESHOLD = 100;
    public static final double CLIMBER_POSITION_TWO = 80;
    public static final double CLIMBER_DOWN_HEIGHT = 2.5;
    public static final double CLIMBER_CLIMB_HEIGHT = 4;
  }

  public static class VisionHardware {
     public static final String CAMERA_B_NAME = "BackAprilTagCam";
    public static final Transform3d CAMERA_B_LOCATION = new Transform3d(
      new Translation3d(0.148, 0.2667, 0.47),
      new Rotation3d(0.0, Math.toRadians(-25.0), Math.toRadians(+180.0))
    );
    public static final Resolution CAMERA_B_RESOLUTION = Resolution.RES_1280_720;
    public static final Rotation2d CAMERA_B_FOV = Rotation2d.fromDegrees(79.7);
  }
  
//is not strong enough to make shot
  public static class PodiumSettings {
    public static final double PODIUM_ARM_ANGLE = 38; //rests at 37 degrees
    public static final double PODIUM_SET_SPEED = 3000;
    public static final double PODIUM_ARM_ANGLE_BACKWARDS = 70;
  }
  public static class BuntSettings {
    public static final double BUNT_ARM_ANGLE = 20; //rests at 37 degrees
    public static final double BUNT_SET_SPEED = 1000;
  }

  public static class ClearanceSettings {
    public static final double CLEARANCE_ARM_ANGLE = 60; //rests at 37 degrees
    public static final double CLEARANCE_SET_SPEED = 3000;
  }

  public static class SubwooferSettings {
    public static final double SUBWOOFER_ARM_ANGLE = 53;
    public static final double SUBWOOFER_SET_SPEED = 3000;
    public static final double SUBWOOFER_ARM_ANGLE_BACKWARDS = 110;
    public static final double SHOOT_FEEDER_SPEED = 4000;
  }

  public static class AmpShotSettings {
    public static final double AMPSHOT_ARM_ANGLE = 128; 
    public static final double AMPSHOT_SET_SPEED = 500;
    public static final double SHOOT_FEEDER_SPEED = 4000;
  }

  public static class ObjectDetection {
    public static final double OBJECTDETECTION_NOTE_KP = 1e-2; 
    public static final double OBJECTDETECTION_NOTE_KI = 0;
    public static final double OBJECTDETECTION_NOTE_KD = 1e-5;
    public static final DoubleConsumer OBJECTDETECTION_NOTE_OFFSET = null;
  }

  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

  public static Optional<Alliance> currentAlliance = null;
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.lasarobotics.drive.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.SDSMK4SwerveModule;
import org.lasarobotics.hardware.kauailabs.NavX2;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.PIDConstants;
import org.lasarobotics.hardware.ctre.PhoenixCANBus;
import org.lasarobotics.hardware.ctre.CANCoder;
import edu.wpi.first.math.util.Units;


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

  public static class HID {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final double CONTROLLER_DEADBAND = 0.10;
    public static final int OPERATOR_CONTROLLER_PORT = 0;
  }

  public static class Drive {
    public static final PIDConstants DRIVE_TURN_PID = new PIDConstants(8.0, 0.0, 0.3, 0.0);
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
    public static final double DRIVE_WIDTH = Units.inchesToMeters(17.32);
    public static final double DRIVE_LENGTH = Units.inchesToMeters(21.5);
  }

  public static class DriveHardware {
    public static final NavX2.ID NAVX_ID = new NavX2.ID("DriveHardware/NavX2");
    public static final Spark.ID LEFT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Drive", 3);
    public static final Spark.ID LEFT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftFront/Rotate", 4);
    public static final Spark.ID RIGHT_FRONT_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Drive", 1);
    public static final Spark.ID RIGHT_FRONT_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightFront/Rotate", 2);
    public static final Spark.ID LEFT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Drive", 5);
    public static final Spark.ID LEFT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/LeftRear/Rotate", 6);
    public static final Spark.ID RIGHT_REAR_DRIVE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Drive", 7);
    public static final Spark.ID RIGHT_REAR_ROTATE_MOTOR_ID = new Spark.ID("DriveHardware/Swerve/RightRear/Rotate", 8);
    public static final CANCoder.ID LEFT_FRONT_ENCODER = new CANCoder.ID("DriveHardware/Swerve/LeftFront/Encoder", PhoenixCANBus.RIO, 32);
    public static final CANCoder.ID LEFT_REAR_ENCODER = new CANCoder.ID("DriveHardware/Swerve/LeftRear/Encoder", PhoenixCANBus.RIO, 34);
    public static final CANCoder.ID RIGHT_FRONT_ENCODER = new CANCoder.ID("DriveHardware/Swerve/RightFront/Encoder", PhoenixCANBus.RIO, 33);
    public static final CANCoder.ID RIGHT_REAR_ENCODER = new CANCoder.ID("DriveHardware/Swerve/RightRear/Encoder", PhoenixCANBus.RIO, 31);
  }

  public static class ArmHardware {
    public static final Spark.ID ARM_MOTOR_ID = new Spark.ID("ArmHardware", 9);
    public static final int ARM_GEAR_RATIO = 64; //FIXME
    public static final int CURRENT_LIMIT_ARM = 30;  
    public static final PIDConstants ARM_ROTATE_PID = new PIDConstants( 0.010086, 0.0, 0.0, 0.0);
    public static final double ARM_TARGET_RPM = 0;
    public static final boolean ARM_ROTATE_SENSOR_PHASE=false;
    public static final boolean ARM_ROTATE_INVERT_MOTOR=false;
    public static final double ARM_ROTATE_TOLERANCE=0.1;
    public static final double ARM_ROTATE_LOWER_LIMIT=0;
    public static final double ARM_ROTATE_UPPER_LIMIT=0;
    public static final boolean ARM_ROTATE_SOFT_LIMITS=false;
    public static final double ARM_KS = 0.38203;
    public static final double ARM_KV = 0.01656;
    public static final double ARM_KA = 0.0813398;
    public static final double ARM_PID_PROFILED_P = 0;
    public static final double ARM_PID_PROFILED_I = 0;
    public static final double ARM_PID_PROFILED_D = 0;
    public static final double ARM_MAX_SPEED = 300;
    public static final double ARM_MAX_ACCELERATION = 300;
    public static final double ARM_POSITION_TOLERANCE = 1;
  }

public static class FeederHardware {
    public static final Spark.ID FEEDER_MOTOR_ID = new Spark.ID("FeederHardware", 11);
    public static final int FEEDER_MOTOR_ONE = 11;
    public static final int FEEDER_CURRENT_LIMIT = 80;
    public static final double FEEDER_TARGET_RPM = 1000;
    public static final boolean FEEDER_SENSOR_PHASE=false;
    public static final boolean FEEDER_INVERT_MOTOR=false;
    public static final double FEEDER_TOLERANCE=0.1;
    public static final PIDConstants FEEDER_PID = new PIDConstants( 9.565E-08, 0.0, 0.0, 0.0);
    public static final double FEEDER_KS = 0.03095;//0.19095;
    public static final double FEEDER_KV = 0.1215 / 1.01;
    public static final double FEEDER_KA = 0.0086947;
}

  public static class ShooterHardware {
    public static final Spark.ID SHOOTER_MOTOR_ID = new Spark.ID("ShooterHardware", 10);
    public static final double SHOOTER_TARGET_RPM = 1000;
    public static final int SHOOTER_CURRENT_LIMIT = 80;
    public static final PIDConstants SHOOTER_PID = new PIDConstants( 1.1159E-05, 0.0, 0.0, 0.0);
    public static final boolean SHOOTER_SENSOR_PHASE=false;
    public static final boolean SHOOTER_INVERT_MOTOR=false;
    public static final double SHOOTER_TOLERANCE=0.1;
    public static final boolean SHOOTER_SOFT_LIMITS=false;
    public static final double SHOOTER_MULTIPLIER = 5800;
    public static final double SHOOTER_KS = 0.06716;
    public static final double SHOOTER_KV = 0.12479;
    public static final double SHOOTER_KA = 0.010604;
  }

  public static class SourceIntakeHardware {
    public static final double INTAKE_SET_ANGLE = -15;
    public static final double INTAKE_SET_SPEED = -4000;
    public static final double INTAKE_FEEDER_SPEED = -1000;
  }
  
//is not strong enough to make shot
  public static class PodiumSettings {
    public static final double PODIUM_ARM_ANGLE = -18;
    public static final double PODIUM_SET_SPEED = 6000;
    public static final double PODIUM_ARM_ANGLE_BACKWARDS = -92;
  }

  public static class SubwooferSettings {
    public static final double SUBWOOFER_ARM_ANGLE = -27;
    public static final double SUBWOOFER_SET_SPEED = 5000;
    public static final double SUBWOOFER_ARM_ANGLE_BACKWARDS = -75;
    public static final double SHOOT_FEEDER_SPEED = 6000;
  }

  public static class DriverHardware {
    public static final Spark.ID DRIVER_MOTOR_ID = new Spark.ID("DriverHardware", 11);

  }
  public static class SmartDashboard {
    public static final String SMARTDASHBOARD_DEFAULT_TAB = "SmartDashboard";
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.autos.DriveStraight;
import frc.team5406.robot.autos.OneGamePiece;
import frc.team5406.robot.commands.AlignWithTag;
import frc.team5406.robot.commands.AlignWithSpeaker;
import frc.team5406.robot.commands.IntakeFromFloor;
import frc.team5406.robot.commands.MoveClimber;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.commands.ShootFromPodium;
import frc.team5406.robot.commands.ShootFromSubwoofer;
import frc.team5406.robot.commands.ShooterOuttake;
import frc.team5406.robot.commands.TrapZero;
import frc.team5406.robot.commands.Unclimb;
import frc.team5406.robot.commands.AmpShot;
import frc.team5406.robot.commands.ArmZero;
import frc.team5406.robot.commands.Outtake;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.ClimbSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.TrapSubsystem;
import frc.team5406.robot.subsystems.IntakeSubsystem;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;
import frc.team5406.robot.subsystems.vision.VisionSubsystem;
import frc.team5406.robot.commands.Bunt;
import frc.team5406.robot.commands.ClearanceShot;
import frc.team5406.robot.commands.Climb;
import frc.team5406.robot.commands.ClimberZero;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class RobotContainer {
  private static final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(
    DriveSubsystem.initializeHardware(),
    Constants.Drive.DRIVE_TURN_PID,
    Constants.Drive.DRIVE_CONTROL_CENTRICITY,
    Constants.Drive.DRIVE_TURN_SCALAR,
    Constants.HID.CONTROLLER_DEADBAND,
    Constants.Drive.DRIVE_LOOKAHEAD,
    Constants.Drive.DRIVE_SLIP_RATIO,
    Constants.Drive.DRIVE_THROTTLE_INPUT_CURVE,
    Constants.Drive.DRIVE_TURN_INPUT_CURVE
  );

    private static final VisionSubsystem VISION_SUBSYSTEM = VisionSubsystem.getInstance();
    CommandXboxController driverController = new CommandXboxController(Constants.HID.DRIVER_CONTROLLER_PORT);
    CommandXboxController  operatorController = new CommandXboxController(Constants.HID.OPERATOR_CONTROLLER_PORT);

    private static SendableChooser<Command> m_automodeChooser = new SendableChooser<>();
      ShooterSubsystem shoot = new ShooterSubsystem();
      FeederSubsystem feeder = new FeederSubsystem();
      IntakeSubsystem intake = new IntakeSubsystem();
      ArmSubsystem arm = new ArmSubsystem();
      TrapSubsystem trap = new TrapSubsystem();
      ClimbSubsystem climber = new ClimbSubsystem();
      Trigger haveNoteTrigger = new Trigger(feeder::getShouldRumble);

  public RobotContainer() {
      shoot.setDefaultCommand(
        new RunCommand( () -> shoot.stopShooter(), shoot));
      feeder.setDefaultCommand(
        new RunCommand( () -> feeder.stopFeeder(), feeder));
      arm.setDefaultCommand(
        new RunCommand(() -> arm.gotoArmAngle(Constants.ArmHardware.ARM_ZERO_ANGLE), arm).unless(()->arm.getClimbing()));
      intake.setDefaultCommand(
        new RunCommand( () -> intake.stopIntake(), intake));

    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
      DRIVE_SUBSYSTEM.driveCommand(
        () -> aboveDeadband(driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
        () -> aboveDeadband(driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
        () -> driverController.getRightX()
      )
    );

    // Setup AutoBuilder
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    // Bind buttons and triggers
    configureBindings();
    defaultShuffleboardTab();
  }

  private static AprilTag speakerSupplier() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue
      ? Constants.BLUE_SPEAKER_TAG
      : Constants.RED_SPEAKER_TAG;
  }

  private void configureBindings() {
    driverController.start().whileTrue(
          new InstantCommand(() -> DRIVE_SUBSYSTEM.resetPose(new Pose2d(DRIVE_SUBSYSTEM.getPose().getX(),DRIVE_SUBSYSTEM.getPose().getY(), Rotation2d.fromDegrees(0.0))))
    );

    operatorController.leftTrigger().and(operatorController.start()).whileTrue(
      new RunCommand( () -> shoot.setShooterSpeed(operatorController.getLeftTriggerAxis()*Constants.ShooterHardware.SHOOTER_MULTIPLIER), shoot)
    );
    
   operatorController.a().whileTrue(
    new ParallelCommandGroup(
      new RunCommand( () -> shoot.setShooterManual(), shoot),
      new RunCommand( () -> arm.setArmAngleManual(), arm)
      )
    );

    operatorController.b().whileTrue(
      new Shoot(feeder) 
    );

    operatorController.rightBumper().whileTrue(
      new ShootFromSubwoofer(arm, shoot, false)
      );

    operatorController.leftBumper().whileTrue(
      new ShootFromPodium (arm, shoot, false)
      );
    
    operatorController.rightBumper().and(operatorController.start()).whileTrue(
      new ShootFromSubwoofer(arm, shoot, true)
      );

    operatorController.leftBumper().and(operatorController.start()).whileTrue(
      new ShootFromPodium (arm, shoot, true)
      );

    driverController.rightTrigger().whileTrue(
      new IntakeFromFloor (feeder, intake)
    );

    driverController.rightBumper().whileTrue(
        new AlignWithTag(VISION_SUBSYSTEM, DRIVE_SUBSYSTEM, 
          () -> aboveDeadband(driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
          () -> aboveDeadband(driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
          () -> driverController.getRightX(),
          6
        )
      );

    driverController.leftBumper().whileTrue(
      new ParallelCommandGroup(
        new AlignWithTag(VISION_SUBSYSTEM, DRIVE_SUBSYSTEM, 
          () -> aboveDeadband(driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
          () -> aboveDeadband(driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
          () -> driverController.getRightX(),
          7
        ),
        new AlignWithSpeaker(arm, VISION_SUBSYSTEM, shoot, () -> speakerSupplier())
      )
    );

    driverController.leftTrigger().whileTrue(
      new Outtake (feeder, intake)
    );

    driverController.leftTrigger().and(driverController.back()).onTrue(
      new ShooterOuttake(feeder, intake, shoot)
    );

    haveNoteTrigger.whileTrue(
      new RunCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0))
      );
      
    haveNoteTrigger.whileFalse(
      new RunCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
    );

    operatorController.a().and(operatorController.back().negate()).onTrue(
      new AmpShot (arm, shoot, feeder)
    );
    
    operatorController.x().and(operatorController.back()).onTrue(
      new ArmZero (arm)
    );

    driverController.y().and(driverController.back().negate()).whileTrue(
      new Bunt (arm, shoot, true)
    );
   
    driverController.y().and(driverController.back()).whileTrue(
      new ClearanceShot (arm, shoot, true)
    );

    driverController.a().whileTrue(
      new ParallelCommandGroup(
        new RunCommand( () -> shoot.setShooterManual(), shoot),
        new RunCommand( () -> arm.setArmAngleManual(), arm)
        )
      );

    operatorController.povRight().and(operatorController.start()).onTrue(
      new MoveClimber(climber, Constants.ClimberHardware.CLIMBER_CLIMB_HEIGHT, Constants.ClimberHardware.CLIMBER_TOLERANCE)
    );

    operatorController.povUp().and(operatorController.start()).onTrue(
      new Unclimb(arm, trap, shoot, feeder, climber)
    );

    operatorController.povLeft().and(operatorController.start()).onTrue(
      new MoveClimber(climber, Constants.ClimberHardware.CLIMBER_POSITION_TWO, Constants.ClimberHardware.CLIMBER_TOLERANCE)
    );

    operatorController.y().and(operatorController.back()).onTrue(
        new TrapZero(trap)
    );

    operatorController.a().and(operatorController.back()).onTrue(
      new ClimberZero(climber)
  );

    operatorController.povDown().and(operatorController.start()).onTrue(
        new Climb(arm, trap, shoot, feeder, climber)
    );
  }

  /**
   * Run simlation related methods
   */
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }

  private void autoModeChooser() {
    m_automodeChooser.setDefaultOption("Do nothing", new SequentialCommandGroup());
    m_automodeChooser.addOption("Simple", new DriveStraight(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("One Game Piece", new OneGamePiece(DRIVE_SUBSYSTEM, arm, shoot, feeder));
  }

  /**
   * Get currently selected autonomous command
   * @return Autonomous command
   */
    public Command getAutonomousCommand() {
      return m_automodeChooser.getSelected();
    }

    public Command getZeroingCommand() {
      return new ClimberZero(climber);
    }

    public void defaultShuffleboardTab() {
      Shuffleboard.selectTab(Constants.SmartDashboard.SMARTDASHBOARD_DEFAULT_TAB);
      autoModeChooser();
      SmartDashboard.putData(Constants.SmartDashboard.SMARTDASHBOARD_AUTO_MODE, m_automodeChooser);
    }

  public double inputScale(double x, double y) {
    double xAbs = Math.abs(x);
    double yAbs = Math.abs(y);
    double angle = Math.atan2(yAbs, xAbs);
    double result = yAbs > xAbs ? Math.sin(angle) : Math.cos(angle);
    return result;
}

   public double aboveDeadband(double input) {
    return (Math.abs(input) > Constants.HID.CONTROLLER_DEADBAND) ? input : 0;
  }
}
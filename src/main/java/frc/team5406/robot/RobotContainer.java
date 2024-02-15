// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team5406.robot.autos.DriveStraight;
import frc.team5406.robot.autos.OneGamePiece;
import frc.team5406.robot.commands.IntakeFromSource;
import frc.team5406.robot.commands.ShootFromPodium;
import frc.team5406.robot.commands.ShootFromSubwoofer;
import frc.team5406.robot.subsystems.ArmSubsystem;
import frc.team5406.robot.subsystems.FeederSubsystem;
import frc.team5406.robot.subsystems.ShooterSubsystem;
import frc.team5406.robot.subsystems.drive.DriveSubsystem;

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

    CommandXboxController driverController =
    new CommandXboxController(Constants.HID.DRIVER_CONTROLLER_PORT);
        CommandXboxController  operatorController =
    new CommandXboxController(Constants.HID.OPERATOR_CONTROLLER_PORT);

    private static SendableChooser<Command> m_automodeChooser = new SendableChooser<>();

      ShooterSubsystem shoot = new ShooterSubsystem();
      FeederSubsystem feeder = new FeederSubsystem();
      ArmSubsystem arm = new ArmSubsystem();

  public RobotContainer() {
      shoot.setDefaultCommand(
        new RunCommand( () -> shoot.stopShooter(), shoot));
        feeder.setDefaultCommand(
          new RunCommand( () -> feeder.stopFeeder(), feeder));
          arm.setDefaultCommand(
            new RunCommand( () -> arm.gotoArmAngle(0), arm));

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

  private void configureBindings() {
    driverController.start().whileTrue(
          new InstantCommand(() -> DRIVE_SUBSYSTEM.resetPose(new Pose2d(DRIVE_SUBSYSTEM.getPose().getX(),DRIVE_SUBSYSTEM.getPose().getY(), Rotation2d.fromDegrees(0.0))))
    );

    operatorController.leftTrigger().and(operatorController.start()).whileTrue(
      new RunCommand( () -> shoot.setShooterSpeed(operatorController.getLeftTriggerAxis()*Constants.ShooterHardware.SHOOTER_MULTIPLIER), shoot)
    );

   /* operatorController.a().whileTrue(
      new RunCommand( () -> shoot.setShooterManual(), shoot)
    );*/

    operatorController.b().whileTrue(
      new RunCommand( () -> feeder.setFeederSpeed(6000), feeder)
    );

   /* operatorController.leftBumper().whileTrue(
      new RunCommand( () -> arm.setArmAngleManual(), arm)
    );*/

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


      operatorController.a().whileTrue(
      new IntakeFromSource (arm, shoot, feeder)
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

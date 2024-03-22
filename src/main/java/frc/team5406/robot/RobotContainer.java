// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team5406.robot;

import java.time.Clock;
import java.util.List;

import com.ctre.phoenix6.Timestamp;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPathLTV;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.team5406.robot.autos.RedAmpSideFourPieceClose;
import frc.team5406.robot.autos.RedAmpSideThreePieceOneFar;
import frc.team5406.robot.autos.RedAmpSideTwoPieceClose;
import frc.team5406.robot.autos.BlueAmpSideFourPieceClose;
import frc.team5406.robot.autos.BlueAmpSideThreePieceOneFar;
import frc.team5406.robot.autos.BlueAmpSideTwoPieceClose;
import frc.team5406.robot.autos.BlueDriveStraight;
import frc.team5406.robot.autos.BlueMiddleThreePieceAmp;
import frc.team5406.robot.autos.BlueMiddleThreePiecePodium;
import frc.team5406.robot.autos.BlueMiddleTwoPieceClose;
import frc.team5406.robot.autos.BlueSourceSideOnePieceClose;
import frc.team5406.robot.autos.BlueSourceSideTwoPieceOneFar;
import frc.team5406.robot.autos.OnePiece;
import frc.team5406.robot.autos.RedDriveStraight;
import frc.team5406.robot.autos.RedMiddleThreePieceAmp;
import frc.team5406.robot.autos.RedMiddleThreePiecePodium;
import frc.team5406.robot.autos.RedMiddleTwoPieceClose;
import frc.team5406.robot.autos.RedSourceSideOnePieceClose;
import frc.team5406.robot.autos.RedSourceSideThreePieceTwoFar;
import frc.team5406.robot.autos.RedSourceSideTwoPieceOneFar;
//import frc.team5406.robot.autos.ThreePieceLocalCenter;
import frc.team5406.robot.autos.RedAmpSideTwoPieceClose;
import frc.team5406.robot.commands.AlignWithTag;
import frc.team5406.robot.commands.AlignForTrap;
import frc.team5406.robot.commands.AlignWithSpeaker;
import frc.team5406.robot.commands.IntakeFromFloor;
import frc.team5406.robot.commands.MoveArmTrapezoid;
import frc.team5406.robot.commands.MoveClimber;
import frc.team5406.robot.commands.Shoot;
import frc.team5406.robot.commands.ShootFromPodium;
import frc.team5406.robot.commands.ShootFromStage;
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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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


    CommandXboxController driverController =
    new CommandXboxController(Constants.HID.DRIVER_CONTROLLER_PORT);
        CommandXboxController  operatorController =
    new CommandXboxController(Constants.HID.OPERATOR_CONTROLLER_PORT);

    private static SendableChooser<Command> m_automodeChooser = new SendableChooser<>();
      ShooterSubsystem shoot = new ShooterSubsystem();
      FeederSubsystem feeder = new FeederSubsystem();
      IntakeSubsystem intake = new IntakeSubsystem();
      ArmSubsystem arm = new ArmSubsystem();
      TrapSubsystem trap = new TrapSubsystem();
      ClimbSubsystem climber = new ClimbSubsystem();
      Trigger haveNoteTrigger = new Trigger(feeder::getShouldRumble);
      Trigger hasSpunUp = new Trigger(shoot::getShouldRumble);


  public RobotContainer() {
      shoot.setDefaultCommand(
        new RunCommand( () -> shoot.stopShooter(), shoot).unless(()->arm.getClimbing()));
        feeder.setDefaultCommand(
          new RunCommand( () -> feeder.stopFeeder(), feeder));
          arm.setDefaultCommand(
              new RunCommand(() -> arm.gotoArmAngle(Constants.ArmHardware.ARM_ZERO_ANGLE), arm).unless(()->arm.getClimbing())
          );
          intake.setDefaultCommand(
            new RunCommand( () -> intake.stopIntake(), intake));

    // Set drive command
    DRIVE_SUBSYSTEM.setDefaultCommand(
      DRIVE_SUBSYSTEM.driveCommand(
        () -> aboveDeadband(driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
        () -> aboveDeadband(driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
        () -> driverController.getRightX(),
        () -> driverController.b().getAsBoolean()
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
    

   operatorController.a().and(operatorController.start()).and(operatorController.back()).whileTrue(
    new ParallelCommandGroup(
      new RunCommand( () -> shoot.setShooterManual(), shoot),
      new RunCommand( () -> arm.setArmAngleManual(), arm)
      )
    );

    
    operatorController.b().whileTrue(
      new Shoot(feeder) 
    );
    

   /* operatorController.leftBumper().whileTrue(
      new RunCommand( () -> arm.setArmAngleManual(), arm)
    );*/
//o

    operatorController.rightBumper().whileTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> arm.gotoArmAngle(Constants.FixedShotSettings.SUBWOOFER_ARM_ANGLE),  arm),
        new RunCommand(() -> shoot.setShooterSpeed(Constants.ShooterHardware.SHOOTER_TARGET_RPM, false), shoot)
      )
      //new ShootFromSubwoofer(arm, shoot)
      //new MoveArmTrapezoid(Constants.SubwooferSettings.SUBWOOFER_ARM_ANGLE, arm)
      );

    operatorController.leftBumper().and(operatorController.back().negate()).whileTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> arm.gotoArmAngle(Constants.FixedShotSettings.PODIUM_ARM_ANGLE),  arm),
        new RunCommand(() -> shoot.setShooterSpeed(Constants.ShooterHardware.SHOOTER_TARGET_RPM, false), shoot)
      )
      //new ShootFromPodium (arm, shoot)
      //new MoveArmTrapezoid(Constants.PodiumSettings.PODIUM_ARM_ANGLE, arm)
      );

    operatorController.leftBumper().and(operatorController.back()).whileTrue(
      new ParallelCommandGroup(
        new RunCommand(() -> arm.gotoArmAngle(Constants.FixedShotSettings.STAGE_ARM_ANGLE),  arm),
        new RunCommand(() -> shoot.setShooterSpeed(Constants.ShooterHardware.SHOOTER_TARGET_RPM, false), shoot)
      )

      //new ShootFromStage (arm, shoot)
      //new MoveArmTrapezoid(Constants.PodiumSettings.PODIUM_ARM_ANGLE, arm)
      );

      driverController.rightTrigger().whileTrue(
      new IntakeFromFloor (feeder, intake)
      );

    driverController.rightBumper().whileTrue(
        new AlignWithTag(VISION_SUBSYSTEM, DRIVE_SUBSYSTEM, 
          () -> aboveDeadband(driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
          () -> aboveDeadband(driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
          () -> driverController.getRightX()*0.6,
          () -> speakerSupplier()
        )
      );

      driverController.leftBumper().whileTrue(
        new ParallelCommandGroup(
          new AlignWithTag(VISION_SUBSYSTEM, DRIVE_SUBSYSTEM, 
            () -> aboveDeadband(driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
            () -> aboveDeadband(driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
            () -> driverController.getRightX()*0.6,
            () -> speakerSupplier()
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

    hasSpunUp.whileTrue(
      new RunCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 1.0))
      );
      
    hasSpunUp.whileFalse(
      new RunCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
    );


    operatorController.a().and(operatorController.back().negate()).onTrue(
      new AmpShot (arm, shoot, feeder)
      );
    
    operatorController.x().and(operatorController.back()).onTrue(
      new ArmZero (arm)
    );

     driverController.y().and(driverController.back().negate()).whileTrue(
      new Bunt (arm, shoot, feeder, true)
      );
   
      driverController.y().and(driverController.back()).whileTrue(
        new ClearanceShot (arm, shoot, feeder, true)
      );


      driverController.a().onTrue(
        AlignForTrap(DRIVE_SUBSYSTEM)
      );

    /*driverController.a().whileTrue(1
        new ParallelCommandGroup(
                  new RunCommand( () -> shoot.setShooterManual(), shoot),
                new RunCommand( () -> arm.setArmAngleManual(), arm)

        )
      );*/
    /*driverController.x().onTrue(
      new RunCommand(() -> trap.goToTrapAngle(20), trap)
    );

    driverController.x().and(driverController.back()).onTrue(
      new RunCommand(() -> trap.goToTrapAngle(1), trap)
    );*/

   /*  driverController.b().onTrue(
      AlignForTrap(DRIVE_SUBSYSTEM)
    );*/


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
    m_automodeChooser.setDefaultOption("0 - Do nothing", new SequentialCommandGroup());
    m_automodeChooser.addOption("1 - Red Drive Straight", new RedDriveStraight(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("2 - Red Source Side, 1 Piece, Close", new RedSourceSideOnePieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder));
    m_automodeChooser.addOption("3 - Red Amp Side, 2 Piece, Close", new RedAmpSideTwoPieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("4 - Red Amp Side, 4 Piece, Close", new RedAmpSideFourPieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("5 - Red Amp Side, 3 Piece, 1 Far", new RedAmpSideThreePieceOneFar(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("6 - Red Source Side, 2 Piece, 1 Far", new RedSourceSideTwoPieceOneFar(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("7 - Red Middle, 2 Piece, Close", new RedMiddleTwoPieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("8 - Red Middle, 3 Piece, Close Amp", new RedMiddleThreePieceAmp(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("9 - Red Middle, 3 Piece, Close Podium", new RedMiddleThreePiecePodium(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));

    m_automodeChooser.addOption("10 - Blue Drive Straight", new BlueDriveStraight(DRIVE_SUBSYSTEM));
    m_automodeChooser.addOption("11 - Blue Source Side, 1 Piece, Close", new BlueSourceSideOnePieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder));
    m_automodeChooser.addOption("12 - Blue Amp Side, 2 Piece, Close", new BlueAmpSideTwoPieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("13 - Blue Amp Side, 4 Piece, Close", new BlueAmpSideFourPieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("14 - Blue Amp Side, 3 Piece, 1 Far", new BlueAmpSideThreePieceOneFar(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("15 - Blue Source Side, 2 Piece, 1 Far", new BlueSourceSideTwoPieceOneFar(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("16 - Blue Middle, 2 Piece, Close", new BlueMiddleTwoPieceClose(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("17 - Blue Middle, 3 Piece, Close Amp", new BlueMiddleThreePieceAmp(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("18 - Blue Middle, 3 Piece, Close Podium", new BlueMiddleThreePiecePodium(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    m_automodeChooser.addOption("19 - Red Source Side, 1 Piece, Close", new OnePiece(DRIVE_SUBSYSTEM, arm, shoot, feeder));
    //m_automodeChooser.addOption("7 - Source Side, 3 Piece, 2 Far", new SourceSideThreePieceTwoFar(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
    //m_automodeChooser.addOption("Three Piece Local Center", new ThreePieceLocalCenter(DRIVE_SUBSYSTEM, arm, shoot, feeder, VISION_SUBSYSTEM, intake, () -> speakerSupplier()));
  }


  public Command AlignForTrap(DriveSubsystem drive) {

    return Commands.runOnce(() -> {
Pose2d currentPose = drive.getPose();
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
      Translation2d move = new Translation2d(Units.inchesToMeters(-9), 0.0).rotateBy(currentPose.getRotation());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(move), currentPose.getRotation());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathConstraints constraints = new PathConstraints(1.0, 1.0, 2 * Math.PI, 2 * Math.PI);

      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        constraints,
        new GoalEndState(0.0, currentPose.getRotation())
      );

      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }
    
    
    );
      
  

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

      public Command stopRumbling() {
        return new ParallelCommandGroup(
            new RunCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0)),
            new RunCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0))
        );
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
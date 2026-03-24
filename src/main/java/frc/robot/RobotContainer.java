// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.NullCommand;
import frc.robot.commands.RevUp;
import frc.robot.commands.ShooterOne;
import frc.robot.commands.TeleOpTurret;
import frc.robot.commands.AutoTurretLeft;
import frc.robot.commands.TeleOpTurret;
import frc.robot.commands.ResetTurret;

//import robot subsystems
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

// We don't need this rn.
// import frc.robot.subsystems.limelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.revrobotics.spark.config.LimitSwitchConfig;
import java.io.IOException;
import java.util.List;

import javax.print.attribute.standard.MediaSize.NA;

import org.json.simple.parser.ParseException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.CancelCommandEvent;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.events.PointTowardsZoneEvent;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;
import com.pathplanner.lib.events.ScheduleCommandEvent;
import com.pathplanner.lib.events.TriggerEvent;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.LocalADStar.GridPosition;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.trajectory.SwerveModuleTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.JSONUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PPLibTesting;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    
    public static boolean climbBottomedOut;

    //up is positive, down is negative
    public static String climbState = "idk";

  // The robot's subsystems
    public static final DriveSubsystem robotDrive = new DriveSubsystem();
    public static final TurretSubsystem TurretSubsystem = new TurretSubsystem();
    public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public static final limelightSubsystem LimeLightSubsystem = new limelightSubsystem();
    public static final ClimbSubsystem climbSubsystem = new ClimbSubsystem();


    // public static final ManualTurretSubsystem ManualTurretSubsystem = new ManualTurretSubsystem();

    //Robot Commands
    public static final DriveCommand driveCommand = new DriveCommand();
    public static final ShooterOne ShooterOne = new ShooterOne();
    public static final ClimbCommand climbCommand = new ClimbCommand();
    public static final AutoShoot autoShoot = new AutoShoot();
    public static final ClimbUp climbUp = new ClimbUp();
    public static final ClimbDown climbDown = new ClimbDown();
    public static final RevUp revUp = new RevUp();
    public static final AutoTurretLeft autoTurretLeft = new AutoTurretLeft();
    public static final NullCommand nullCommand = new NullCommand();
  

    

  // The driver's controller
  public static XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static Joystick operatorController = new Joystick(OIConstants.kOperatorControllerPort); 
  public static CommandXboxController m_commandXboxController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static CommandJoystick m_operatorController = new CommandJoystick(OIConstants.kOperatorControllerPort);
  

  private final SendableChooser<Command> autoChooser;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  

  public RobotContainer() {

     NamedCommands.registerCommand("Reset Gyro", new InstantCommand(
        () -> robotDrive.zeroHeading()
      ));
    
      //ZPaths 
    NamedCommands.registerCommand("Hello, World!", autonPath("Hello, World!"));
    
    //Path Planner Commands
    NamedCommands.registerCommand("AutoShoot", autoShoot);
    NamedCommands.registerCommand("RevUp", revUp);
    NamedCommands.registerCommand("AutoTurret", autoTurretLeft);
    NamedCommands.registerCommand("NullCommand", nullCommand);

    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Select Auto", autoChooser);

   // new RunCommand(() -> , null)
    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true), 
              robotDrive ));
        
              // LimeLightSubsystem.setDefaultCommand(
              //   new ShooterOne()
              // );

              

  }





/**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> robotDrive.setX(),
    //         robotDrive));
      m_commandXboxController.pov(180).whileTrue(
        new RunCommand(
          () -> robotDrive.zeroHeading(), robotDrive)
      );

      m_operatorController.axisLessThan(3, 0.8).whileTrue(
        ShooterOne);


      //m_commandXboxController.a().onTrue(climbCommand);
      m_commandXboxController.a().whileTrue(climbDown);
      m_commandXboxController.b().whileTrue(climbUp);
      m_operatorController.button(3).whileTrue(teleOpTurret);
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */



  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

   public Command autonPath(String pathname) {
    PathPlannerPath path = null;
      try {
        path = PathPlannerPath.fromPathFile(pathname);
    }
     catch (IOException | ParseException e) {
      // Auto-generated catch block
      e.printStackTrace();
     }

     boolean isConfigured = AutoBuilder.isConfigured();
     if (isConfigured) 
        System.out.println(pathname + " is configured");
     else
        System.out.println(pathname + " is not configured");
  
    return AutoBuilder.followPath(path);
  }
  //   // Create config for trajectory
  //   TrajectoryConfig config = new TrajectoryConfig(
  //       AutoConstants.kMaxSpeedMetersPerSecond,
  //       AutoConstants.kMaxAccelerationMetersPerSecondSquared)
  //       // Add kinematics to ensure max speed is actually obeyed
  //       .setKinematics(DriveConstants.kDriveKinematics);

  //       return autoChooser.getSelected();

    // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     ,
    //     robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     robotDrive::setModuleStates,
    //     robotDrive);

    // Reset odometry to the starting pose of the trajectory.


    // Run path following command, then stop at the end.
   // return swerveControllerCommand.andThen(() -> robotDrive.drive(0, 0, 0, false));
  }


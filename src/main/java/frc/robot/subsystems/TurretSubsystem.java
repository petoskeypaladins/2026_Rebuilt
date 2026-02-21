package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// REV Robotics imports
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;


//@SuppressWarnings("unused")
public class TurretSubsystem extends SubsystemBase {
  private final SparkMax m_turretRotate = new SparkMax(
      Constants.MechConstants.turretRotateCanID, MotorType.kBrushless);
  private final RelativeEncoder m_turretEncoder = m_turretRotate.getEncoder();
  private final SparkClosedLoopController m_turretController = m_turretRotate.getClosedLoopController();
    private double speed;
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5); // Adjust rate as needed
    boolean resettinghigh = false;
    boolean resettinglow = false;
     private boolean wasEnabled = false;
     double turretLimit = 16;
     double turretRatio = 360 / 16.90; // <-- adjust as needed
  
  
    /** Creates a new TurretSubsystem. */
    public TurretSubsystem() {

    resetTurretEncoder();
  
    }
  
    @Override
    
    public void periodic() {
  SmartDashboard.putNumber("TurretPosition", m_turretEncoder.getPosition());
  SmartDashboard.putNumber("TurretVelocity", m_turretEncoder.getVelocity());
      boolean isEnabled = DriverStation.isEnabled();
      if (isEnabled && !wasEnabled) {
          resetTurretEncoder();
      }
      wasEnabled = isEnabled;
  
  if (m_turretEncoder.getPosition() >= turretLimit ) {
      resettinghigh = true;
  }

  if (m_turretEncoder.getPosition() <= -turretLimit ) {
    resettinglow = true;
  }
    }

  
  
    public double getTurretPosition() {
      return m_turretEncoder.getPosition();
    }
  
    public void resetTurretEncoder() {
      m_turretEncoder.setPosition(0);
    
    }
  

  public void setTurretSpeed(){
    turretTrackPose(null);
  }

public void turretTrackPose (Pose2d target)
{
    // Get robot pose with turret offset
    Pose2d robotPose = RobotContainer.robotDrive.getPose();
    // Apply turret offset to robot pose
    Transform2d turretOffset = 
      new Transform2d(
        new Translation2d(Constants.TurretConstants.kTurretTransformInchesX / 39.37, Constants.TurretConstants.kTurretTransformInchesY / 39.37), 
        new Rotation2d(0));
    Pose2d turretPose =( robotPose.plus(turretOffset));
    

    double turretAngle = (turretPose.getRotation().getDegrees())+turretRatio * m_turretEncoder.getPosition();

   System.out.println("turretAngle" + turretAngle);
   System.out.println("RobotX" + robotPose.getX());
   System.out.println("RobotY" + robotPose.getY());
   System.out.println("RobotRotation" + robotPose.getRotation().getDegrees());
    SmartDashboard.putNumber("TurretAngle", turretAngle);
    SmartDashboard.putNumber("TurretOffsetX", turretOffset.getX());
    SmartDashboard.putNumber("TurretOffsetY", turretOffset.getY());
    SmartDashboard.putNumber("Turret Pose X", turretPose.getX());
    SmartDashboard.putNumber("Turret Pose Y", turretPose.getY());
    SmartDashboard.putNumber("Turret Pose Rotation", turretPose.getRotation().getDegrees());
    SmartDashboard.putNumber("BotPose X", robotPose.getX());
    SmartDashboard.putNumber("BotPose Y", robotPose.getY());
    SmartDashboard.putNumber("BotPose Rotation", robotPose.getRotation().getDegrees());
}
}

  


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

import java.security.PublicKey;

import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;

// @SuppressWarnings("unused")
public class TurretSubsystem extends SubsystemBase {


  public static final SparkMax turretRotate = new SparkMax(
      Constants.MechConstants.turretRotateCanID, MotorType.kBrushless);
  public static final RelativeEncoder m_turretEncoder = turretRotate.getEncoder();
  public static final SparkClosedLoopController m_turretController = turretRotate.getClosedLoopController();
  //m_turretController.setSetpoint(setPoint, ControlType.
    private double speed;
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5); // Adjust rate as needed
    boolean resettinghigh = false;
    boolean resettinglow = false;
     private boolean wasEnabled = false;
     double turretLimit = 16;
     double turretRatio = 360 / 16.90; // <-- adjust as needed
      
      public double m_robotRelativeAngle;
     public double m_fieldRelativeAngle;
  
    //* Creates a new TurretSubsystem.
    public TurretSubsystem() {
            m_turretEncoder.setPosition(0);
    }
    
    
  

    @Override
    
    public void periodic() {
  SmartDashboard.putNumber("TurretPosition", m_turretEncoder.getPosition());
  SmartDashboard.putNumber("TurretVelocity", m_turretEncoder.getVelocity());
  SmartDashboard.putBoolean("resettingHigh", resettinghigh);
  SmartDashboard.putBoolean("ResettingLow", resettinglow);
       boolean isEnabled = DriverStation.isEnabled();
       if (isEnabled && !wasEnabled) {
           resetTurretEncoder();
           resettinghigh = false;
           resettinglow = false;
       }
      wasEnabled = isEnabled;
  
  if (m_turretEncoder.getPosition() >= turretLimit ) {
      resettinghigh = true;
  }

  if (m_turretEncoder.getPosition() <= -turretLimit ) {
    resettinglow = true;
  }

if (resettinghigh == false && resettinglow == false) {
  setTurretSpeed(RobotContainer.operatorController.getRawAxis(2) * 0.3);
} else {
  if (resettinghigh == true)  {
      setTurretSpeed(-0.1);
    if (m_turretEncoder.getPosition() <= 0) {
      resettinghigh = false;
    }
  } else if (resettinglow == true) {
      setTurretSpeed(0.1);
      if (m_turretEncoder.getPosition() >= 0){
        resettinglow = false;
    }
  }
}
  
}

  
  
    public double getTurretPosition() {
      return m_turretEncoder.getPosition();
    }
  
    public void resetTurretEncoder() {
      m_turretEncoder.setPosition(0);
    
    }
  

  public void setTurretSpeed(double speed){
   turretRotate.set(speed);

  }


// public void turretTrackPose (Pose2d target)
// {
//     // Get robot pose with turret offset
//     Pose2d robotPose = RobotContainer.robotDrive.getPose();
//     // Apply turret offset to robot pose
//     Transform2d turretOffset = 
//       new Transform2d(
//         new Translation2d(Constants.TurretConstants.kTurretTransformInchesX / 39.37, Constants.TurretConstants.kTurretTransformInchesY / 39.37), 
//         new Rotation2d(0));
        
//     Pose2d turretPose = (robotPose.plus(turretOffset));
    

//     double turretAngle = (turretPose.getRotation().getDegrees())+turretRatio * m_turretEncoder.getPosition();

// ////#region Positions and stuff on the dashboard
//     SmartDashboard.putNumber("TurretAngle", turretAngle);
//     SmartDashboard.putNumber("TurretOffsetX", turretOffset.getX());
//     SmartDashboard.putNumber("TurretOffsetY", turretOffset.getY());
//     SmartDashboard.putNumber("Turret Pose X", turretPose.getX());
//     SmartDashboard.putNumber("Turret Pose Y", turretPose.getY());
//     SmartDashboard.putNumber("Turret Pose Rotation", turretPose.getRotation().getDegrees());
//     SmartDashboard.putNumber("BotPose X", robotPose.getX());
//     SmartDashboard.putNumber("BotPose Y", robotPose.getY());
//     SmartDashboard.putNumber("BotPose Rotation", robotPose.getRotation().getDegrees());



     
//     // Calculate vector to target
//     double dY = target.getY() - turretPose.getY();
//     double dX = target.getX() - turretPose.getX();
    
//     // Calculate field-relative angle to target
//     Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(Math.atan2(dY, dX));

//     // Convert to robot-relative angle
//     Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(robotPose.getRotation());

//     // Update angle variables
//     m_robotRelativeAngle = robotRelativeAngle.getDegrees();
//     m_fieldRelativeAngle = fieldRelativeAngle.getDegrees();

//     SmartDashboard.putNumber("m_robotRelativeAngle", m_robotRelativeAngle);
//     SmartDashboard.putNumber("m_fieldRelativeAngle", m_fieldRelativeAngle);
//     // Command the turret

//     //Line 145 is *allegedly* the problem that causes the code to not enable. : (
//     //m_turretEncoder.setPosition(m_robotRelativeAngle*turretRatio);

    
//     SmartDashboard.putNumber("Turret Target Position", m_turretEncoder.getPosition());
    

//     //we need to convert the desired ankle of the turret to a position of the motor.
    

    
// }
}

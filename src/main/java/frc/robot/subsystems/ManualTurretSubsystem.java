
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
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

 @SuppressWarnings("unused")
public class ManualTurretSubsystem extends SubsystemBase {


 
  //m_turretController.setSetpoint(setPoint, ControlType.
    private double speed;
    private final SlewRateLimiter slewRateLimiter = new SlewRateLimiter(0.5); // Adjust rate as needed
    boolean resettinghigh = false;
    boolean resettinglow = false;
     private boolean wasEnabled = false;
     double turretLimit = 16;
     double turretRatio = 360 / 16.90; // <-- adjust as needed
  
  
    //* Creates a new TurretSubsystem.
    public ManualTurretSubsystem() {

    resetTurretEncoder();
  
    }
  
    @Override
    
    public void periodic() {

  boolean isEnabled = DriverStation.isEnabled();
      if (isEnabled && !wasEnabled) {
          resetTurretEncoder();
      } //reset the turret encoder on enable
      wasEnabled = isEnabled;

  SmartDashboard.putNumber(getName(), TurretSubsystem.m_turretEncoder.getPosition());
  SmartDashboard.putNumber(getName(), TurretSubsystem.m_turretEncoder.getVelocity());
  if (TurretSubsystem.m_turretEncoder.getPosition() >= turretLimit ) {
      resettinghigh = true;
  }

  if (TurretSubsystem.m_turretEncoder.getPosition() <= -turretLimit ) {
    resettinglow = true;
  }
    }

  
  
    public double getTurretPosition() {
      return TurretSubsystem.m_turretEncoder.getPosition();
    }
  
    public void resetTurretEncoder() {
      TurretSubsystem.m_turretEncoder.setPosition(0);
    
    }
  

  public void setTurretSpeed(){
if (resettinghigh == true){
    if (Math.abs(TurretSubsystem.m_turretEncoder.getPosition()) >= ( 0.25*turretLimit)){
TurretSubsystem.m_turretRotate.set(-0.3);
    } else {
      TurretSubsystem.m_turretRotate.set(-0.1);
}
    
if (TurretSubsystem.m_turretEncoder.getPosition() <= 0){
resettinghigh = false;

}}  
if (resettinglow == true){
  TurretSubsystem.m_turretRotate.set(0.3);
  if (TurretSubsystem.m_turretEncoder.getPosition() <= (-0.25*turretLimit)) {
TurretSubsystem.m_turretRotate.set(0.3);
    } else {
      TurretSubsystem.m_turretRotate.set(0.1);
}


}
if (resettinghigh == false && resettinglow == false) {
if (RobotContainer.operatorController.getRawButton(3)){
TurretSubsystem.m_turretRotate.set(-0.1);
} else {
 if (RobotContainer.operatorController.getRawButton(4)){
  TurretSubsystem.m_turretRotate.set(0.1);
 } else {
TurretSubsystem.m_turretRotate.set(0);
}

}
}
  }
}
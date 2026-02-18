package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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


@SuppressWarnings("unused")
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
  
  
    /** Creates a new TurretSubsystem. */
    public TurretSubsystem() {

    resetTurretEncoder();
  
    }
  
    @Override
    
    public void periodic() {

      boolean isEnabled = DriverStation.isEnabled();
      if (isEnabled && !wasEnabled) {
          resetTurretEncoder();
      }
      wasEnabled = isEnabled;
  SmartDashboard.putNumber(getName(), m_turretEncoder.getPosition());
  SmartDashboard.putNumber(getName(), m_turretEncoder.getVelocity());
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
if (resettinghigh == true){
    if (Math.abs(m_turretEncoder.getPosition()) >= ( 0.25*turretLimit)){
m_turretRotate.set(-0.3);
    } else {
      m_turretRotate.set(0.3*(0-(m_turretEncoder.getPosition())));
}
    
if (m_turretEncoder.getPosition() <= 0){
resettinghigh = false;

}}
if (resettinglow == true){
  m_turretRotate.set(0.3);
  if ( m_turretEncoder.getPosition() >=0) {
    resettinglow = false;
}


}
if (resettinghigh == false && resettinglow == false) {
if (RobotContainer.operatorController.getRawButton(3)){

m_turretRotate.set(-0.1);
} else {
 if (RobotContainer.operatorController.getRawButton(4)){
  m_turretRotate.set(0.1);
 } else {
m_turretRotate.set(0);
 }
}}
}
}



  

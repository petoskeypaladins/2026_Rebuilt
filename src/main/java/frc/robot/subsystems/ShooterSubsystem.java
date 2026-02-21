
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax shooterBottom = new SparkMax(Constants.MechConstants.ShooterBottomCanID, MotorType.kBrushless);
  private final SparkMax shooterTop = new SparkMax(Constants.MechConstants.ShooterTopCanID, MotorType.kBrushless);
  private final SparkMax spindexer = new SparkMax(Constants.MechConstants.SpindexerCanID, MotorType.kBrushless);
  private final SparkMax kicker = new SparkMax(Constants.MechConstants.KickerCanID, MotorType.kBrushless);

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {}
  public boolean shooterRunning = false;



  double spindexerSpeed = 0;
  double kickerSpeed = 0;
  double shooterTopSpeed = 0;
  double shooterBottomSpeed = 0;

  //Change these when testing speed
  double SPINDEXER_POWER = 0.5;
  double KICKER_POWER = 0.5;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //spindexer and kicker
    if (RobotContainer.operatorController.getPOV() == 270 && shooterRunning == true){
        spindexerSpeed = SPINDEXER_POWER;
        kickerSpeed = KICKER_POWER;
;
    } else if (RobotContainer.operatorController.getPOV() == 90 && shooterRunning == true){
      spindexerSpeed = 0-SPINDEXER_POWER;
      kickerSpeed = 0-KICKER_POWER;
    } else {
      spindexerSpeed = 0;
      kickerSpeed = 0;
    }

  if (RobotContainer.operatorController.getRawButton(1)) {
    shooterRunning = true;
  } else {
    shooterRunning = false;
  }

  

    SmartDashboard.putNumber("Spindexer speed", spindexerSpeed);
    SmartDashboard.putNumber("kicker Speed", kickerSpeed);
    SmartDashboard.putNumber("Shooter top speed", shooterTopSpeed);
    SmartDashboard.putNumber("Shooter bottom speed", shooterBottomSpeed);


    }

  }
  
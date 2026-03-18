
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {

  public final SparkMax shooterBottom = new SparkMax(Constants.MechConstants.ShooterBottomCanID, MotorType.kBrushless);
  public final SparkMax shooterTop = new SparkMax(Constants.MechConstants.ShooterTopCanID, MotorType.kBrushless);
  public final SparkMax spindexer = new SparkMax(Constants.MechConstants.SpindexerCanID, MotorType.kBrushless);
  public final SparkMax kicker = new SparkMax(Constants.MechConstants.KickerCanID, MotorType.kBrushless);
  

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {}
  //public boolean shooterRunning = false;

  boolean testMode;

  double spindexerSpeed = 0;
  double kickerSpeed = 0;


  //Change these when testing speed
  double SPINDEXER_POWER = 0.4;
  double KICKER_POWER = 0.95;

double topManualPower = 0;
double bottomManualPower = 0;
double kickerManualPower = 0;

public boolean forceSlowMode = false;

  @Override
  public void periodic() {


    if (RobotContainer.operatorController.getRawButtonPressed(6)){
      if (testMode == false){
        testMode = true;
      } else {
        testMode = false;
      }

    }
    SmartDashboard.putBoolean("Manual Mode/testMode", testMode);
    if (testMode) {
       kicker.set(kickerManualPower);
    spindexer.set(spindexerSpeed);
    shooterTop.set(topManualPower);
    shooterBottom.set(bottomManualPower);
    } else {
    kicker.set(kickerSpeed);
    spindexer.set(spindexerSpeed);
    // shooterTop.set(shooterTopSpeed);
    // shooterBottom.set(shooterBottomSpeed);

    }


    //#region "manual powers: TEMPORARY!"
    if (RobotContainer.operatorController.getRawButtonPressed(10)){
      topManualPower += 0.025;
    }

        if (RobotContainer.operatorController.getRawButtonPressed(12)){
      topManualPower -= 0.025;
    }

            if (RobotContainer.operatorController.getRawButtonPressed(9)){
      bottomManualPower += 0.025;
    }

            if (RobotContainer.operatorController.getRawButtonPressed(11)){
      bottomManualPower -= 0.025;
    }

          if (RobotContainer.operatorController.getRawButtonPressed(5)){
            kickerManualPower += 0.025;
          }

          if (RobotContainer.operatorController.getRawButtonPressed(3)){
            kickerManualPower -= 0.025;
          }


    SmartDashboard.putNumber("Manual Mode/Top manual power", topManualPower);
    SmartDashboard.putNumber("Manual Mode/Bottom manual power", bottomManualPower);
    SmartDashboard.putNumber("Manual Mode/Kicker manual power", kickerManualPower);
//#endregion


    // This method will be called once per scheduler run
    
    //spindexer and kicker
    if (RobotContainer.operatorController.getRawButton(1) && RobotContainer.ShooterOne.shooterRunning == true){
        spindexerSpeed = SPINDEXER_POWER;
        kickerSpeed = KICKER_POWER;
    } else if (RobotContainer.operatorController.getRawButton(2) && RobotContainer.operatorController.getRawButton(1) ){
      spindexerSpeed = -0.5 * SPINDEXER_POWER; //Reduces power in reverse
      kickerSpeed = -0.5 * KICKER_POWER;  //Reduces power in reverse
    } else {
      spindexerSpeed = 0;
      kickerSpeed = 0;

    }

    SmartDashboard.putNumber("Mechanisims/Spindexer speed", spindexerSpeed);
    SmartDashboard.putNumber("Mechanisims/kicker Speed", kickerSpeed);


    }

  }
  
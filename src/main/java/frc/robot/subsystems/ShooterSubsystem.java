
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@SuppressWarnings("unused")
public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax shooterTop = new SparkMax(Constants.MechConstants.ShooterTopCanID, MotorType.kBrushless);
  private final SparkMax shooterBottom = new SparkMax(Constants.MechConstants.ShooterBottomCanID, MotorType.kBrushless);
  // private final RelativeEncoder m_intakeEncoder = m_intake.getEncoder();
  // private final SparkClosedLoopController m_intakeController = m_intake.getClosedLoopController();

  /** Creates a new IntakeSubsystem. */
  public ShooterSubsystem() {}
  public boolean shooterRunning = false;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    if (RobotContainer.operatorController.getTriggerPressed() == true) {
      if (shooterRunning == true) {
        shooterRunning = false;
        // shooterTop.set(0);
        // shooterBottom.set(0);
        System.out.println("Shooter off :(");
      } else {
        shooterRunning = true;
        // shooterTop.set(0.25);
        // shooterBottom.set(0.25);
        System.out.println("Shooter on!!");
      }
    };

    
  }
}

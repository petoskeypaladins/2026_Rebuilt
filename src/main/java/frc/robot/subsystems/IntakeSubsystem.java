// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkMax intake = new SparkMax(Constants.MechConstants.IntakeCanID, MotorType.kBrushless);
  private final RelativeEncoder m_intakeEncoder = intake.getEncoder();
  private final SparkClosedLoopController m_intakeController = intake.getClosedLoopController();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  double intakeSpeed = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  intake.set(intakeSpeed);

SmartDashboard.putNumber("IntakeSpeed", intakeSpeed);
    if (intakeSpeed == 0){
      if (RobotContainer.driverController.getLeftBumperButtonPressed() == true){
        intakeSpeed = -0.5;
      } else if(RobotContainer.driverController.getRightBumperButtonPressed() == true) {
        intakeSpeed = 0.5;
      }
      
    } else if ((RobotContainer.driverController.getLeftBumperButtonPressed()) || (RobotContainer.driverController.getRightBumperButtonPressed())){
      intakeSpeed = 0;
    }

    
    
  }
}

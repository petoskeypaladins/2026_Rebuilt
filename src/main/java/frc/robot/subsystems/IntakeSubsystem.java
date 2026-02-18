// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {

  // private final SparkMax intake = new SparkMax(Constants.MechConstants.IntakeCanID, MotorType.kBrushless);
  // private final RelativeEncoder m_intakeEncoder = m_intake.getEncoder();
  // private final SparkClosedLoopController m_intakeController = m_intake.getClosedLoopController();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}
  public boolean intakeRunning = false;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    if (RobotContainer.driverController.getRightBumperButtonPressed() == true) {
      if (intakeRunning == true) {
        intakeRunning = false;
        // intake.set(0);
        System.out.println("intake off :(");
      } else {
        intakeRunning = true;
        // intake.set(0.5);
        System.out.println("intake on!!");
      }
    };

    
  }
}

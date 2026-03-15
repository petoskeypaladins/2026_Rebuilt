// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbConstants;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;




public class ClimbSubsystem extends SubsystemBase {
  
  
  /** Creates a new ClimbSubsystem. */

  public DigitalInput limitSwitch = new DigitalInput(ClimbConstants.limitSwitchPort);

  public ClimbSubsystem() {
        
        if (RobotContainer.climbBottomedOut) {
      RobotContainer.climbState = "down";
    } else {
      RobotContainer.climbState = "up";
    }
  }

  public static TalonFX climb = new TalonFX(Constants.ClimbConstants.krakenPortID);
  public static double climbTop = 95;

  @Override
  public void periodic() {

    if (limitSwitch.get() == true) {
      RobotContainer.climbBottomedOut = false;
    } else {
      RobotContainer.climbBottomedOut = true;
      ClimbSubsystem.climb.setPosition(0);
    }






    // This method will be called once per scheduler run
    SmartDashboard.putString("ClimbSubsystem/climb state", RobotContainer.climbState);
    SmartDashboard.putNumber("ClimbSubsystem/Climb encoder", climb.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("ClimbSubsystem/Climb bottomed out?", RobotContainer.climbBottomedOut);
    SmartDashboard.putBoolean("ClimbSubsystem/limit switch", limitSwitch.get());
  }
}




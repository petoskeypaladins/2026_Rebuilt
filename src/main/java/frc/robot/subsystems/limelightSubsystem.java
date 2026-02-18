// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;

@SuppressWarnings("unused")
public class limelightSubsystem extends SubsystemBase {


  /**  Creates a new limelightSubsystem. */
  public limelightSubsystem() {}

  @Override
  public void periodic() {
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = table.getEntry("tx").getDouble(0.0);
    double ty = table.getEntry("ty").getDouble(0.0);
    double ta = table.getEntry("ta").getDouble(0.0);

    SmartDashboard.putNumber("Limelight TX", tx);
    SmartDashboard.putNumber("Limelight TY", ty);
    SmartDashboard.putNumber("Limelight TA", ta);
    // System.out.println("Limelight TX: " + tx + " TY: " + ty + " TA: " + ta);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.*;

@SuppressWarnings("unused")
public class limelightSubsystem extends SubsystemBase {

  public double tx;
  public double ty;
  public double ta;
  public double tid;
  //List <Number> validTags;
  boolean goodtag;
  boolean areTagsFiletered = false;

  /**  Creates a new limelightSubsystem. */
  public limelightSubsystem() {
    
    //filter valid april tags by alliance
    Optional<Alliance> alliance =
    DriverStation.getAlliance();
    if (alliance.isPresent()){
      if (alliance.get() == Alliance.Red){
      //validTags.addAll(Constants.TagConstants.redTags);
      areTagsFiletered = true;
      }
      if (alliance.get() == Alliance.Blue){
      //validTags.addAll(Constants.TagConstants.blueTags);
      areTagsFiletered = true;
      }
    } else {
      //validTags.addAll(Constants.TagConstants.blueTags);
      //validTags.addAll(Constants.TagConstants.redTags);
      areTagsFiletered = false;
    
  }
   }

  @Override
  public void periodic() {
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   tx = table.getEntry("tx").getDouble(0.0);
   ty = table.getEntry("ty").getDouble(0.0);
   ta = table.getEntry("ta").getDouble(0.0);
   tid = table.getEntry("tid").getDouble(ta);

    SmartDashboard.putNumber("Limelight/Limelight TX", tx);
   // SmartDashboard.putNumber("Limelight TY", ty);
    SmartDashboard.putNumber("Limelight/Limelight TA", ta);
    SmartDashboard.putNumber("Limelight/Limelight TID", tid);
    SmartDashboard.putBoolean("Limelight/Is that a good april tag?", goodtag);
    //SmartDashboard.putString("April tag whitelist", validTags.toString());
    SmartDashboard.putBoolean("Limelight/Are the april tags filtered by alliance?", areTagsFiletered);

    //if (validTags.contains(tid)){
    //  goodtag = true;
   // } else {
    //  goodtag = false;
   // }

    // System.out.println("Limelight TX: " + tx + " TY: " + ty + " TA: " + ta);
  }
}

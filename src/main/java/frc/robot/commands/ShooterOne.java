// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.limelightSubsystem;
import java.util.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterOne extends Command {
  /** Creates a new ShooterOne. */
  public ShooterOne() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    addRequirements(RobotContainer.LimeLightSubsystem);

    
  }

  public static boolean shooterRunning = true;
  public String ABtesting = "A";
  public String Adescription = "none";
  public String Bdescription = "none";
  public String CurrentDescription = "null";


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Limelight TY",RobotContainer.LimeLightSubsystem.ty);
    SmartDashboard.putNumber("Shooter top speed", RobotContainer.shooterSubsystem.shooterTop.get());
    SmartDashboard.putNumber("Shooter bottom speed",RobotContainer.shooterSubsystem.shooterBottom.get());
    // SmartDashboard.putString("AB testing", ABtesting);
    // SmartDashboard.putString("Description", CurrentDescription);
    
    //AB testing
    // if (RobotContainer.operatorController.getRawButtonPressed(4)){
    //   if (ABtesting == "A"){
    //     ABtesting = "B";
    //   } else {
    //     ABtesting = "A";
    //   }
    // }

    // if (ABtesting == "A"){
    //   CurrentDescription = Adescription;
    // }

    // if (ABtesting == "B"){
    //   CurrentDescription = Bdescription;
    // }


    double ty = RobotContainer.LimeLightSubsystem.ty;
    if (ty != 0){ //if it sees an april tag
  
      // if (ABtesting == "A"){
      // Adescription = "linear";


     RobotContainer.shooterSubsystem.shooterTop.set(-0.034*ty+0.453);
     RobotContainer.shooterSubsystem.shooterBottom.set(0.011*ty-0.548);
     shooterRunning = true;
      }
         if (RobotContainer.operatorController.getRawButton(11)){
      RobotContainer.shooterSubsystem.shooterTop.set(0.5);
     RobotContainer.shooterSubsystem.shooterBottom.set(-0.3);


    shooterRunning = true;
      }

    //  } else {
      // Bdescription = "cubic";
      // RobotContainer.shooterSubsystem.shooterTop.set(-0.000205039*ty*ty*ty+0.00695272*ty*ty-0.0932683*ty+0.568311);
      // RobotContainer.shooterSubsystem.shooterBottom.set(-(0.000292744*ty*ty*ty+0.0081225*ty*ty-0.0727252*ty+0.658441));
      }
   // }

    
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterRunning = false;
    RobotContainer.shooterSubsystem.shooterTop.set(0);
    RobotContainer.shooterSubsystem.shooterBottom.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

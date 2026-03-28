// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.limelightSubsystem;
import java.util.*;

import javax.lang.model.util.ElementScanner14;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterTwo extends Command {
  /** Creates a new ShooterTwo. */
  public ShooterTwo() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooterSubsystem);
    addRequirements(RobotContainer.LimeLightSubsystem);
  }

  //variable declaration
  public double lastSpeedTop = 0.4;
  public double lastSpeedBottom = -0.4;
  double ty;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (RobotContainer.operatorController.getRawButton(1)){
        RobotContainer.shooterSubsystem.kicker.set(0.3); 
        RobotContainer.shooterSubsystem.spindexer.set(0.4);
    } else if (RobotContainer.operatorController.getRawButton(2)){
        RobotContainer.shooterSubsystem.kicker.set(-0.3);
        RobotContainer.shooterSubsystem.spindexer.set(-0.4);
    } else {
        RobotContainer.shooterSubsystem.kicker.set(0);
        RobotContainer.shooterSubsystem.spindexer.set(0);
    }
  
    ty = RobotContainer.LimeLightSubsystem.ty;
    if (ty != 0){
      lastSpeedTop = -0.010*ty + 0.316;
      lastSpeedBottom = 0.013*ty - 0.504;
    }

    if (RobotContainer.operatorController.getRawButton(11)){
        RobotContainer.shooterSubsystem.shooterTop.set(0.4);
        RobotContainer.shooterSubsystem.shooterBottom.set(-0.4);

    } else if (RobotContainer.operatorController.getRawButton(12)){
        RobotContainer.shooterSubsystem.shooterTop.set(0.8);
        RobotContainer.shooterSubsystem.shooterBottom.set(-0.6);

    } else {
      RobotContainer.shooterSubsystem.shooterTop.set(lastSpeedTop);
      RobotContainer.shooterSubsystem.shooterBottom.set(lastSpeedBottom);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooterSubsystem.shooterTop.set(0);
    RobotContainer.shooterSubsystem.shooterBottom.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

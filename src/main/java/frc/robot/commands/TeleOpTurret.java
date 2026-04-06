// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.limelightSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleOpTurret extends Command {
  /** Creates a new TeleOpTurret. */
  public TeleOpTurret() {
    addRequirements(RobotContainer.TurretSubsystem);
    addRequirements(RobotContainer.LimeLightSubsystem);
  }

  // upper limit (right): +3
  // Lower limit (left): -5

  public double upperLimit = 3.0;
  public double lowerLimit = -5.0;
  public double tx;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.LimeLightSubsystem.tx != 0){ // <- Prevents "zero jitter"
      tx = RobotContainer.LimeLightSubsystem.tx;
    }

    // if (RobotContainer.operatorController.getRawButton(3)){
    // TurretSubsystem.setTurretSpeed(RobotContainer.operatorController.getRawAxis(2)
    // * 0.3);
    // } else {
    // TurretSubsystem.setTurretSpeed(0);
    // }
    if (RobotContainer.operatorController.getRawButton(3)) {
      if (TurretSubsystem.m_turretEncoder.getPosition() <= upperLimit
          && RobotContainer.operatorController.getRawAxis(2) >= 0.05) {
        TurretSubsystem.setTurretSpeed(RobotContainer.operatorController.getRawAxis(2) * 0.3);
      } else if (TurretSubsystem.m_turretEncoder.getPosition() >= lowerLimit
          && RobotContainer.operatorController.getRawAxis(2) <= -0.05) {
        TurretSubsystem.setTurretSpeed(RobotContainer.operatorController.getRawAxis(2) * 0.3);
      } else {
        TurretSubsystem.setTurretSpeed(0);
      }

    } else if (RobotContainer.operatorController.getRawButton(4)) {
      if (tx < 3 && tx > -3){ // if the tx is within the acceptable range...
        TurretSubsystem.setTurretSpeed(0); //set its speed to zero
      } //otherwise...
      else if (tx < -3 ) { // if the turret is to the right of the aprilTag...
        if (TurretSubsystem.m_turretEncoder.getPosition() >= lowerLimit) {
          // auto adjust the turret in the minus direction
          TurretSubsystem.setTurretSpeed(-0.05);
        } else {
          TurretSubsystem.setTurretSpeed(0);
        }
      } else if (tx > 3) { // if the turret is to the left of the aprilTag...
        if (TurretSubsystem.m_turretEncoder.getPosition() <= upperLimit) {
          // auto adjust the turret in the positive direction, aka clockwise, aka to the
          // right
          TurretSubsystem.setTurretSpeed(0.05);
        } else {
          TurretSubsystem.setTurretSpeed(0);
        }
      }
    } else {
      TurretSubsystem.setTurretSpeed(0); //set speed back down to zero when the buttons are released
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TurretSubsystem.setTurretSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleOpTurret extends Command {
  /** Creates a new TeleOpTurret. */
  public TeleOpTurret()
   {
   addRequirements(RobotContainer.TurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


      

    
  
  // if (m_turretEncoder.getPosition() >= turretLimit ) {
  //     resettinghigh = true;
  // }

  // if (m_turretEncoder.getPosition() <= -4.1 ) {
  //   setTurretSpeed(0);
  //   resettinghigh = true;
  // }

if (TurretSubsystem.resettinghigh == false &&TurretSubsystem. resettinglow == false) {
  if (RobotContainer.operatorController.getRawButton(3)){
  TurretSubsystem.setTurretSpeed(RobotContainer.operatorController.getRawAxis(2) * 0.3);
  } else {
    TurretSubsystem.setTurretSpeed(0);
  }
}
// } else {
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTurretLeft extends Command {
  /** Creates a new AutoTurret. */
  public AutoTurretLeft() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.TurretSubsystem);
  }
boolean done = false;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

    double tick = 0;


  // Called every time the scheduler runs while the command is scheduled.
  @Override


  public void execute() {

    

    if (TurretSubsystem.m_turretEncoder.getPosition() >= -3.85) {
      TurretSubsystem.turretRotate.set(-0.3);
      done = false;

    } else {
      TurretSubsystem.turretRotate.set(0);
      done = true;
    }

    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}

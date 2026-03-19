//#region "old with goofy logic"

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import org.json.simple.parser.ParseException;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.ClimbSubsystem;
// import frc.robot.subsystems.DriveSubsystem;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import java.io.IOException;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ClimbCommand extends Command {
//   /** Creates a new ClimbCommand. */
//   /** {@link RobotContainer} */

//   final PositionVoltage request = new PositionVoltage(0).withSlot(0);

//   public ClimbCommand() {

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(
//         RobotContainer.climbSubsystem

//     );
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {



//     /** update the {@link climbBottomedOut} boolean based on the limit switch */

//     if (RobotContainer.climbState == "up" || RobotContainer.climbState == "stopped on the way up") {
//       lowerClimb();
//     } else if (RobotContainer.climbState == "down" || RobotContainer.climbState == "stopped on the way down") {
//       raiseClimb();
//     } else if (RobotContainer.climbState == "going up" || RobotContainer.climbState == "going down") {
      
//       //stopClimb();

//     } else {
//       System.out.println("uh-oh");
//     }


//     // RobotContainer.climbSubsystem.climb.set(0.1);

//   }

//   // Called once the command ends or is interrupted.
//   @SuppressWarnings("static-access")
//   @Override

//   public void end(boolean interrupted) {
//     RobotContainer.climbSubsystem.climb.set(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @SuppressWarnings("static-access")
//   public void raiseClimb() {
//     if (RobotContainer.climbSubsystem.climb.getPosition().getValueAsDouble() < ClimbSubsystem.climbTop) {
//       RobotContainer.climbSubsystem.climb.set(0.3);
//       inbetweenState();
//     } else {
//       RobotContainer.climbSubsystem.climb.set(0);
//       toggleState();
//       cancel();
//     }
//   }

//   @SuppressWarnings("static-access")
//   public void lowerClimb() {
//     if (!RobotContainer.climbBottomedOut) {
//       RobotContainer.climbSubsystem.climb.set(-0.3);
//       inbetweenState();
//     } else {
//       RobotContainer.climbSubsystem.climb.set(0);
//       toggleState();
//       cancel();
//     }
//   }

//   public void toggleState() {
//     if (RobotContainer.climbState == "up") {
//       RobotContainer.climbState = "down";
//     } else {
//       RobotContainer.climbState = "up";
//     }
//   }

//   @SuppressWarnings("static-access")
//   public void inbetweenState() {
//     if (RobotContainer.climbSubsystem.climb.get() >= 0.1){
//       RobotContainer.climbState = "going up";
//     }  
//     else if (RobotContainer.climbSubsystem.climb.get() <= -0.1) {
//       RobotContainer.climbState = "going down";

//     } else {
//       System.out.println("whoopsie");
//     }
//   }


//     public void stoppedState() {
//       if (RobotContainer.climbState == "going up") {
//         RobotContainer.climbState = "stopped on the way up";
//       }
//       if (RobotContainer.climbState == "going down") {
//         RobotContainer.climbState = "stopped on the way down";
//       }
//     }


// @SuppressWarnings("static-access")
// public void stopClimb() {
//   RobotContainer.climbSubsystem.climb.set(0);
//   stoppedState();
//   cancel();
// }


// }

//#endregion

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.json.simple.parser.ParseException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.io.IOException;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbCommand extends Command {
  /** Creates a new ClimbCommand. */
  /** {@link RobotContainer} */

  final PositionVoltage request = new PositionVoltage(0).withSlot(0);

  public ClimbCommand() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(
        RobotContainer.climbSubsystem

    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



    /** update the {@link climbBottomedOut} boolean based on the limit switch */

    if (RobotContainer.climbState == "up") {
      lowerClimb();
    } else if (RobotContainer.climbState == "down") {
      raiseClimb();
    } else {
      System.out.println("uh-oh");
    }

    // RobotContainer.climbSubsystem.climb.set(0.1);

  }

  // Called once the command ends or is interrupted.
  @SuppressWarnings("static-access")
  @Override

  public void end(boolean interrupted) {
    RobotContainer.climbSubsystem.climb.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @SuppressWarnings("static-access")
  public void raiseClimb() {
    if (RobotContainer.climbSubsystem.climb.getPosition().getValueAsDouble() < ClimbSubsystem.climbTop) {
      RobotContainer.climbSubsystem.climb.set(0.5);  // test that, we had it at 0.3
    } else {
      RobotContainer.climbSubsystem.climb.set(0);
      toggleState();
      cancel();
    }
  }

  @SuppressWarnings("static-access")
  public void lowerClimb() {
    if (!RobotContainer.climbBottomedOut) {
      RobotContainer.climbSubsystem.climb.set(-0.3);
    } else {
      RobotContainer.climbSubsystem.climb.set(0);
      toggleState();
      cancel();
    }
  }

  public void toggleState() {
    if (RobotContainer.climbState == "up") {
      RobotContainer.climbState = "down";
    } else {
      RobotContainer.climbState = "up";
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.commands.DriveAutoCommand;
import frc.robot.commands.Autonomous.commands.IntakeAutoCommand;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html 
public class Autodeneme extends SequentialCommandGroup {
  /** Creates a new Autodeneme. */
  public Autodeneme(DifferentialDriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> intakeSubsystem.setAngle(-0.2)).withTimeout(1.7),   
      new RunCommand(() -> intakeSubsystem.setAngle(0)).withTimeout(0.1),
      new DriveAutoCommand(driveSubsystem, -0.6).withTimeout(1.5),
      new IntakeAutoCommand(intakeSubsystem, -0.15).withTimeout(1)
    );  
  }
  

  }

  

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SensDriveCommand extends Command {
  /** Creates a new SensDriveCommand. */
  public DifferentialDriveSubsystem driveSubsystem;
  public double leftSpeed;
  public double rightSpeed;
  
  public SensDriveCommand(DifferentialDriveSubsystem driveSubsystem, double leftSpeed, double rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

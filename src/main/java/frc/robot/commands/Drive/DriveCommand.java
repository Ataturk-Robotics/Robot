// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {

  DriveSubsystem mDriveSubsystem;
  double speed;

  /** Creates a new DriveCommand. */
  public DriveCommand(double _speed, DriveSubsystem subsystem) {
    mDriveSubsystem = subsystem;
    speed = _speed;
    addRequirements(mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDriveSubsystem.setRightMotor(speed);
    mDriveSubsystem.setLeftMotor(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveSubsystem.setRightMotor(0);
    mDriveSubsystem.setLeftMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

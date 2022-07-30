// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotSubsystems.DriveSubsystem;

public class RotateCommand extends CommandBase {

  DriveSubsystem mDriveSubsystem;
  double rotationSpeed;

  /** Creates a new RotateCommand. */
  public RotateCommand(double _rotationSpeed, DriveSubsystem subsystem) {
    mDriveSubsystem = subsystem;
    rotationSpeed = _rotationSpeed;
    addRequirements(mDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDriveSubsystem.setRightMotor(-rotationSpeed);
    mDriveSubsystem.setLeftMotor(rotationSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

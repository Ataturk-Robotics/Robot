// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotSubsystems.DriveSubsystem;

public class DifferentialDriveCommand extends CommandBase {
  private DriveSubsystem mDriveSubsystem;

  /** Creates a new DriveCommand. */
  public DifferentialDriveCommand(DriveSubsystem subsystem) {
    mDriveSubsystem = subsystem;
    addRequirements(mDriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDriveSubsystem.setRightMotor(
        -(Constants.controller.getRawAxis(Constants.yAxis) +
            Constants.controller.getRawAxis(Constants.xAxis)));
    mDriveSubsystem.setLeftMotor(
        -(Constants.controller.getRawAxis(Constants.yAxis)
            - Constants.controller.getRawAxis(Constants.xAxis)));

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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

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
    // TODO: Question the mechanical about their sketchy wiring of the motors
    // Why the hell to move the robot forward we need to set the right to -1 and the left to 1 ???????
    // Is the right motor wired the wrong way around ??????
    // Magic thing i came up with 2 years ago. This needs to change
    mDriveSubsystem.setRightMotor(
        Constants.controller.getRawAxis(Constants.xAxis) +
            Constants.controller.getRawAxis(Constants.yAxis));
    mDriveSubsystem.setLeftMotor(
        Constants.controller.getRawAxis(Constants.xAxis) -
            Constants.controller.getRawAxis(Constants.yAxis));

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

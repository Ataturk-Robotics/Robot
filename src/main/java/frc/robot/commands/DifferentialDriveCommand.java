// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DifferentialDriveCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  /** Creates a new DifferentialDriveCommand. */
  public DifferentialDriveCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(this.driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* driveSubsystem.tankDrive(
    -(JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxyAxis) -
    JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxXAxis)), 
    -(JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxyAxis)
      + JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxXAxis))); */

    driveSubsystem.tankDrive(
    (JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxyAxis) -
    JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxXAxis)), 
    -(JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxyAxis)
    + JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxXAxis)));
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DifferentialDriveCommand extends Command {
  /** Creates a new DifferentialDriveCommand. */
  private DifferentialDriveSubsystem DriveSubsystem;

  public DifferentialDriveCommand(DifferentialDriveSubsystem DriveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DriveSubsystem = DriveSubsystem;
    addRequirements(this.DriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSubsystem.tankDrive(
    (JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxyAxis) -
    JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxXAxis)) * 0.8, 
    (JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxyAxis)
    + JoystickConstants.xBoxController.getRawAxis(JoystickConstants.xBoxXAxis)) * 0.8);

    double sensSpeed = 0.2;
    int povValue = JoystickConstants.xBoxController.getPOV();

    if(povValue == 0){
      DriveSubsystem.tankDrive(sensSpeed, sensSpeed);
    }else if(povValue == 90){
      DriveSubsystem.tankDrive(sensSpeed, -sensSpeed);
    }else if(povValue == 180){
      DriveSubsystem.tankDrive(-sensSpeed, -sensSpeed);
    }else if(povValue == 270){
      DriveSubsystem.tankDrive(-sensSpeed, sensSpeed);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

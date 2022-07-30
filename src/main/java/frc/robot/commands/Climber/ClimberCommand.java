// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotSubsystems.ClimberSubsystem;

public class ClimberCommand extends CommandBase {

  private ClimberSubsystem mClimberSubsystem;

  /** Creates a new ClimberCommand. */
  public ClimberCommand(ClimberSubsystem subsystem) {
    mClimberSubsystem = subsystem;
    addRequirements(mClimberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = -Constants.controller.getRawAxis(Constants.powerAxis);
    if (speed > 0) {

      if (mClimberSubsystem.toplimitSwitch.get()) {
        mClimberSubsystem.setClimberMotors(speed);
      } else {
        mClimberSubsystem.setClimberMotors(0);
      }

    } else {

      if (mClimberSubsystem.bottomlimitSwitch.get()) {
        mClimberSubsystem.setClimberMotors(speed);
      } else {
        mClimberSubsystem.setClimberMotors(0);
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimberSubsystem.setClimberMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {

  TurretSubsystem turretSubsystem;

  /** Creates a new TurretCommand. */
  public TurretCommand(TurretSubsystem turretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turretSubsystem;
    addRequirements(this.turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turretSubsystem.setTurretMotor(-0.75 * JoystickConstants.ps4Controller.getRawAxis(JoystickConstants.Ps4XAxis));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turretSubsystem.setTurretMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

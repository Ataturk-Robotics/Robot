// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AngleCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  double targetPosition;
  PIDController pidController;

  /** Creates a new AngleCommand. */
  public AngleCommand(IntakeSubsystem intakeSubsystem, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.targetPosition = targetPosition;
    
    pidController = new PIDController(0.1, 0, 0);

    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = intakeSubsystem.getEncoder();

    double output = pidController.calculate(currentPosition);

    output = Math.max(0.1, Math.min(0.1, output));

    intakeSubsystem.setAngle(-output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setAngle(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}

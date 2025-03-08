// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.commands;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnAngleCommand extends Command{
  private DifferentialDriveSubsystem driveSubsystem;
  private AHRS gyro;
  private PIDController pidController;
  private double targetAngle;
  
  public TurnAngleCommand(DifferentialDriveSubsystem driveSubsystem, AHRS gyro, double targetAngle){
    this.driveSubsystem = driveSubsystem;
    this.gyro =  gyro;
    this.targetAngle = targetAngle;
    this.pidController = new PIDController(0.027, 0, 0);

    pidController.setTolerance(1);
    addRequirements(driveSubsystem);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public void execute() {
    double output = pidController.calculate(gyro.getAngle());
    driveSubsystem.tankDrive(-0.3 * output, 0.3 * output);
  }

  @Override
  public void initialize() {
    pidController.setSetpoint(targetAngle);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  
}

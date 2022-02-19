// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive.DifferentialDriveCommand;

public class DriveSubsystem extends SubsystemBase {

  WPI_VictorSPX rightMotor = new WPI_VictorSPX(Constants.kRightMotorId);
  WPI_VictorSPX leftMotor = new WPI_VictorSPX(Constants.kLeftMotorId);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    setDefaultCommand(new DifferentialDriveCommand(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setRightMotor(double value) {
    rightMotor.set(ControlMode.PercentOutput, value);
  }

  public void setLeftMotor(double value) {
    leftMotor.set(ControlMode.PercentOutput, value);
  }

  public void driveForwardForSeconds(double speed, double seconds) {
    // TODO
    throw new UnsupportedOperationException("Not implemented yet");
  }

  public void rotateForSeconds(double speed, double seconds) {
    // TODO
    throw new UnsupportedOperationException("Not implemented yet");
  }
}

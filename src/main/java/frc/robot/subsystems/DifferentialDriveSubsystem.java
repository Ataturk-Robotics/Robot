// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.commands.DriveCommands.DifferentialDriveCommand;

public class DifferentialDriveSubsystem extends SubsystemBase {
  WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(MotorIDConstants.leftMotorIds[0]);
  WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(MotorIDConstants.leftMotorIds[1]);

  WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(MotorIDConstants.rightMotorIds[0]);
  WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(MotorIDConstants.rightMotorIds[1]);

  //public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  public final Encoder encoder = new Encoder(0, 1);

  private final DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);

  /** Creates a new DifferentialDriveSubsystem. */
  public DifferentialDriveSubsystem() {
    setDefaultCommand(new DifferentialDriveCommand(this));

    rearLeftMotor.follow(frontLeftMotor);
    rearRightMotor.follow(frontRightMotor);

    frontLeftMotor.setInverted(true);
  }


  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stop() {
    tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

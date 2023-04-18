// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
// import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DifferentialDriveCommand;

public class DriveSubsystem extends SubsystemBase {

  WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(DriveConstants.leftMotorIds[0]);
  WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(DriveConstants.leftMotorIds[1]);

  WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(DriveConstants.rightMotorIds[0]);
  WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(DriveConstants.rightMotorIds[1]);

  private final DifferentialDrive m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
   setDefaultCommand(new DifferentialDriveCommand(this));

   rearLeftMotor.follow(frontLeftMotor);
   rearRightMotor.follow(rearRightMotor);

   frontLeftMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  
  public void tankDrive(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void stop() {
    tankDrive(0, 0);
  }
  
}

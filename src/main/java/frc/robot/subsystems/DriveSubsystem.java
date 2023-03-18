// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AtarobGyro;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DifferentialDriveCommand;

public class DriveSubsystem extends SubsystemBase {

  WPI_VictorSPX frontLeftMotor = new WPI_VictorSPX(DriveConstants.leftMotorIds[0]);
  WPI_VictorSPX rearLeftMotor = new WPI_VictorSPX(DriveConstants.leftMotorIds[1]);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(
    frontLeftMotor, rearLeftMotor
  );

  WPI_VictorSPX frontRightMotor = new WPI_VictorSPX(DriveConstants.rightMotorIds[0]);
  WPI_VictorSPX rearRightMotor = new WPI_VictorSPX(DriveConstants.rightMotorIds[1]);

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(
    frontRightMotor, rearRightMotor
  );

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder
  private final Encoder m_leftEncoder = new Encoder(
    DriveConstants.kLeftEncoderPorts[0],
    DriveConstants.kLeftEncoderPorts[1],
    DriveConstants.kLeftEncoderReversed
  );

  // The right-side drive encoder
  private final Encoder m_rightEncoder = new Encoder(
    DriveConstants.kRightEncoderPorts[0],
    DriveConstants.kRightEncoderPorts[1],
    DriveConstants.kRightEncoderReversed
  );

  private final AtarobGyro gyro = new AtarobGyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
   setDefaultCommand(new DifferentialDriveCommand(this));

   m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
   m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

   resetEncoders();
   odometry =
   new DifferentialDriveOdometry(
    gyro.getRotation2d(),
    m_leftEncoder.getDistance(),
    m_rightEncoder.getDistance()
   );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(
      gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public void drive(double leftSpeed, double rightSpeed) {
    frontLeftMotor.set(ControlMode.PercentOutput, -leftSpeed);
    rearLeftMotor.set(ControlMode.PercentOutput, leftSpeed);

    frontRightMotor.set(ControlMode.PercentOutput, -rightSpeed);
    rearRightMotor.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void stop() {
    drive(0, 0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }


  
}

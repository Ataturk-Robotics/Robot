// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.RobotSubsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive.DifferentialDriveCommand;

public class DriveSubsystem extends SubsystemBase {

  WPI_VictorSPX rightMotor = new WPI_VictorSPX(Constants.kRightMotorId);
  WPI_VictorSPX leftMotor = new WPI_VictorSPX(Constants.kLeftMotorId);

  DifferentialDrive drive = new DifferentialDrive(rightMotor, leftMotor);

  private Encoder leftEncoder = new Encoder(
    Constants.kLeftEncoderPorts[0],
    Constants.kLeftEncoderPorts[1],
    Constants.kLeftEncoderReversed
  );

  private Encoder rightEncoder = new Encoder(
    Constants.kRightEncoderPorts[0],
    Constants.kRightEncoderPorts[1],
    Constants.kRightEncoderReversed
  );

  private EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);


  private final AnalogGyro gyro = new AnalogGyro(1);

  static final double KvLinear = 1.98;
  static final double KaLinear = 0.2;
  static final double KvAngular = 1.5;
  static final double KaAngular = 0.3;

  DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),
    7.29,
    7.5,
    60.0,
    Units.inchesToMeters(3),
    0.7112,

    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
  );

  private final Field2d field = new Field2d();

  private AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);

  private final DifferentialDriveOdometry odometry;

  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0));

  

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    rightMotor.setInverted(true);

    leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    SmartDashboard.putData("Field", field);

    var wheelSpeeds = new DifferentialDriveWheelSpeeds(2.0, 3.0);

    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);


    setDefaultCommand(new DifferentialDriveCommand(this));
  }


  @Override
  public void periodic(){

    simulationPeriodic();

    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    field.setRobotPose(odometry.getPoseMeters());
  }

  public void simulationPeriodic(){
    driveSim.setInputs(
      leftMotor.get() * RobotController.getInputVoltage(),
      rightMotor.get() * RobotController.getInputVoltage()
    );

    driveSim.update(0.02);

    leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-driveSim.getHeading().getDegrees());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public double getLeftRate(){
    return leftEncoder.getRate();
  }

  public double getRightRate(){
    return rightEncoder.getRate();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    //drive(leftVolts, rightVolts);

    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    gyro.reset();
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -gyro.getRate();
  }

  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  public void setRightMotor(double value) {
    rightMotor.set(ControlMode.PercentOutput, value);
  }

  public void setLeftMotor(double value) {
    leftMotor.set(ControlMode.PercentOutput, value);
  }

  public void drive(double leftValue, double rightValue){   
    setLeftMotor(leftValue);
    setRightMotor(rightValue);
  }
  public void stop(){
    setLeftMotor(0);
    setRightMotor(0);
  }

}

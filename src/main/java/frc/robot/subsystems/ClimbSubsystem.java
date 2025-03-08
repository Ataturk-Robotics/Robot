// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  SparkMax climbMotor = new SparkMax(MotorIDConstants.ClimbingMotorID, MotorType.kBrushed);
  SparkMax grabberMotor = new SparkMax(MotorIDConstants.GrabberMotorID, MotorType.kBrushed);

  public ClimbSubsystem() {}

  public void setClimbingMotor(double speed) {
    climbMotor.set(speed);
  }

  public void setGrabberMotor(double speed){
      grabberMotor.set(-speed);
  }  

  public void stop() {
    climbMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

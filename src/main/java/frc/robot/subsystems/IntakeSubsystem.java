// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.commands.Intake.IntakeCommand;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  SparkMax neoMotor = new SparkMax(MotorIDConstants.IntakeNeoMotorID, MotorType.kBrushless);
  SparkMax cimMotor = new SparkMax(MotorIDConstants.IntakeCimMotorID, MotorType.kBrushless);
  SparkMax angleMotor = new SparkMax(MotorIDConstants.IntakeAngleMotorID, MotorType.kBrushless);

  RelativeEncoder angleEncoder;
  
  public IntakeSubsystem() {
    setDefaultCommand(new IntakeCommand(this));
    angleEncoder = angleMotor.getEncoder();
  }

  public double getEncoder(){
    return angleEncoder.getPosition();
  }

  public void setMotor(double speed) {
    neoMotor.set(speed);
    cimMotor.set(-speed);
  }

  public void setAngle(double speed){
    angleMotor.set(-speed);
  }

  public void stop() {
    neoMotor.set(0);
    cimMotor.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getEncoder());
  }
}

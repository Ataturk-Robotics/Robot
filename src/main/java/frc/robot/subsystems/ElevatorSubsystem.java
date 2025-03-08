// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDConstants;
import frc.robot.commands.ElevatorCommand;

public class ElevatorSubsystem extends SubsystemBase {
    public final DigitalInput topLimitSwitch;
    public final DigitalInput bottomLimitSwitch;

  /** Creates a new ElevatorSubsystem. */
  SparkMax motor1 = new SparkMax(MotorIDConstants.ElevatorMotorID[0], MotorType.kBrushed);
  SparkMax motor2 = new SparkMax(MotorIDConstants.ElevatorMotorID[1], MotorType.kBrushed);

  public ElevatorSubsystem() {
    setDefaultCommand(new ElevatorCommand(this));
      topLimitSwitch = new DigitalInput(4);
      bottomLimitSwitch = new DigitalInput(5);
  }

  public void setMotors(double speed) {
    /* if((speed > 0 && !topLimitSwitch.get()) ||
       (speed < 0 && !bottomLimitSwitch.get())) {
        stop();
    } */
    motor1.set(-speed);
    motor2.set(-speed);
  }

  public void stop() {
      motor1.set(0);
      motor2.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

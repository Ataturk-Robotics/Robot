// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  WPI_VictorSPX climberMotors[] = {
      new WPI_VictorSPX(Constants.climberMotorIds[0])// , new WPI_VictorSPX(Constants.climberMotorIds[1])
  };

  public DigitalInput toplimitSwitch = new DigitalInput(0);
  public DigitalInput bottomlimitSwitch = new DigitalInput(1);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setClimberMotors(double speed) {
    for (WPI_VictorSPX motor : climberMotors) {
      motor.set(ControlMode.PercentOutput, speed);
    }
  }
}

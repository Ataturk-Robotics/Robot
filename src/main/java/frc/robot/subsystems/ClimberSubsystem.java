// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

  CANSparkMax climberMotors[] = {
      new CANSparkMax(Constants.climberMotorIds[0], MotorType.kBrushed),
      new CANSparkMax(Constants.climberMotorIds[1], MotorType.kBrushed)
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
    climberMotors[0].set(-speed);
    climberMotors[1].set(speed);
  }
}

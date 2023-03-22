// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.AngleCommand;

public class AngleSubsystem extends SubsystemBase {

  WPI_VictorSPX angleMotors[] = {
    new WPI_VictorSPX(SubsystemConstants.angleMotorIds[0]),
    new WPI_VictorSPX(SubsystemConstants.angleMotorIds[1])
  };

  public DigitalInput topLimitSwitch = new DigitalInput(0);
  public DigitalInput bottomLimitSwitch = new DigitalInput(1);


  /** Creates a new AngleSubsystem. */
  public AngleSubsystem() {
    setDefaultCommand(new AngleCommand(this));
  }

  public void setAngleMotor(double speed){
    angleMotors[0].set(-speed);
    angleMotors[1].set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.ArmCommand;

public class ArmSubsystem extends SubsystemBase {

  WPI_VictorSPX armMotors[] = {
    new WPI_VictorSPX(SubsystemConstants.armMotorIds[0]),
    new WPI_VictorSPX(SubsystemConstants.armMotorIds[1])
  };

  public DigitalInput topLimitSwitch = new DigitalInput(2);
  public DigitalInput bottomLimitSwitch = new DigitalInput(3);

  /** Creates a new ArmSubsystem. */

  public ArmSubsystem() {
    setDefaultCommand(new ArmCommand(this));
  }

  public void setArmMotor(double speed){
    armMotors[0].set(ControlMode.PercentOutput, -speed);
    armMotors[1].set(ControlMode.PercentOutput, -speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

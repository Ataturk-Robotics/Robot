// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.TurretCommand;

public class TurretSubsystem extends SubsystemBase {

  WPI_VictorSPX turretMotor = new WPI_VictorSPX(SubsystemConstants.turretMotorId);
  //AtarobGyro  gyro = new AtarobGyro();

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    turretMotor.setInverted(true);
    setDefaultCommand(new TurretCommand(this));
  }

  public void setTurretMotor(double speed){
    turretMotor.set(ControlMode.PercentOutput, -speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

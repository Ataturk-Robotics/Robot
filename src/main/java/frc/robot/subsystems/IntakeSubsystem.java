// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final DoubleSolenoid armSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.CTREPCM,
      Constants.kIntakeSolenoidPorts[0],
      Constants.kIntakeSolenoidPorts[1]);

  WPI_VictorSPX rollerSpx = new WPI_VictorSPX(Constants.rollerMotorId);
  WPI_VictorSPX intakeSpx = new WPI_VictorSPX(Constants.intakeMotorId);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Intake Periodic");
  }

  public Value getArmValue() {
      return armSolenoid.get();
  }

  public void dropArm() {
    armSolenoid.set(Value.kForward);
  }

  public void liftArm() {
    armSolenoid.set(Value.kReverse);
  }

  public void setRoller(double value){
    rollerSpx.set(ControlMode.PercentOutput, value);
  }

  public void setIntake(double value){
    intakeSpx.set(ControlMode.PercentOutput, value);
  }


}

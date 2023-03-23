// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {

  WPI_VictorSPX intake = new WPI_VictorSPX(SubsystemConstants.intakeMotorId);

  private final DoubleSolenoid armSolenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, 
    0, 
    2);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void setIntakeMotor(double speed){
    intake.set(-speed);
  }

  public Value getIntakeValue(){
    return armSolenoid.get();
  }

  public void openIntake(){
    armSolenoid.set(Value.kForward);
  }
  
  public void closeIntake(){
    armSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

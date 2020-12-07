/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class C_Asilma extends Command {
  public C_Asilma() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.sAsilma);
  }

  WPI_VictorSPX asoshlarMotor = new WPI_VictorSPX(17);
  WPI_VictorSPX takoshlarMotor = new WPI_VictorSPX(16);

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  boolean asoshlar = false;
  boolean takoshlar = false;

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.asoshlarBaslaButton.get()){
      asoshlar = true;}
    if(Robot.m_oi.asoshlarBitirButton.get()){
      asoshlar = false;
    }
    if(asoshlar){
      asoshlarMotor.set(ControlMode.PercentOutput, Robot.m_oi.getDriverRawAxis(RobotMap.sliderAxis));
    }
    else{
      asoshlarMotor.set(ControlMode.PercentOutput, 0);
    }

    if(Robot.m_oi.takoshlarBaslaButton.get()){
      takoshlar = true;}
    if(Robot.m_oi.takoshlarBitirButton.get()){
      takoshlar = false;
    }
    if(takoshlar){
      takoshlarMotor.set(ControlMode.PercentOutput, -Robot.m_oi.getDriverRawAxis(RobotMap.sliderAxis));
    }
    else{
      takoshlarMotor.set(ControlMode.PercentOutput, 0);
    }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

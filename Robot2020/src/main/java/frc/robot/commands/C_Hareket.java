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

public class C_Hareket extends Command {
  public C_Hareket() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.sHareket);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  WPI_VictorSPX sagMotor = new WPI_VictorSPX(11);
  WPI_VictorSPX solMotor = new WPI_VictorSPX(10);
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    sagMotor.set(ControlMode.PercentOutput, +Robot.m_oi.getDriverRawAxis(RobotMap.yAxis) + Robot.m_oi.getDriverRawAxis(RobotMap.xAxis));
    solMotor.set(ControlMode.PercentOutput, -Robot.m_oi.getDriverRawAxis(RobotMap.yAxis) + Robot.m_oi.getDriverRawAxis(RobotMap.xAxis));
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

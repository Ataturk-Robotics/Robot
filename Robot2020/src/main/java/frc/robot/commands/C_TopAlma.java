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

public class C_TopAlma extends Command {
  public C_TopAlma() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.sTopAlma);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  public WPI_VictorSPX topuslarMotor = new WPI_VictorSPX(RobotMap.topAlmaMotorID);

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.topuslarButton.get()){
      topuslarMotor.set(ControlMode.PercentOutput, -Robot.m_oi.getDriverRawAxis(RobotMap.sliderAxis));
    }
    else{
      topuslarMotor.set(ControlMode.PercentOutput, 0);
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

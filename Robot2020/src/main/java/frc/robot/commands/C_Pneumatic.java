/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class C_Pneumatic extends Command {
  public C_Pneumatic() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.sPneumatic);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  Compressor com = new Compressor(0);
  DoubleSolenoid dsol = new DoubleSolenoid(2, 4);
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.compressorBaslaButton.get()){com.start();}
    if(Robot.m_oi.compressorBitirButton.get()){com.stop();dsol.set(Value.kOff);}
    if(Robot.m_oi.compressorIleriButton.get()){dsol.set(Value.kForward);}
    if(Robot.m_oi.compressorGeriButton.get()){dsol.set(Value.kReverse);}
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

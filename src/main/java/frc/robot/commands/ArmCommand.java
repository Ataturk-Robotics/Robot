// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {


  ArmSubsystem armSubsystem;

  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(this.armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -JoystickConstants.ps4Controller.getRawAxis(JoystickConstants.ps4ZAxis) * 0.75;
    if(speed < 0){
      if(armSubsystem.topLimitSwitch.get()){
        armSubsystem.setArmMotor(speed);
      }else{
        armSubsystem.setArmMotor(0);
      }
    }else{
      if(armSubsystem.bottomLimitSwitch.get()){
        armSubsystem.setArmMotor(speed);
      }else{
        armSubsystem.setArmMotor(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

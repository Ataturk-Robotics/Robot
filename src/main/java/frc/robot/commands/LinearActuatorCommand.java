// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.LinearActuatorSubsystem;

public class LinearActuatorCommand extends CommandBase {

  LinearActuatorSubsystem linearSubsystem;

  /** Creates a new AngleCommand. */
  public LinearActuatorCommand(LinearActuatorSubsystem linearSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.linearSubsystem = linearSubsystem;
    addRequirements(this.linearSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -JoystickConstants.ps4Controller.getRawAxis(JoystickConstants.Ps4yAxis);
    if(speed < 0){
      if(linearSubsystem.topLimitSwitch.get()){
        linearSubsystem.setAngleMotor(speed);
      }else{
        linearSubsystem.setAngleMotor(0);
      }
    }else{
      if(linearSubsystem.bottomLimitSwitch.get()){
        linearSubsystem.setAngleMotor(speed);
      }else{
        linearSubsystem.setAngleMotor(0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linearSubsystem.setAngleMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

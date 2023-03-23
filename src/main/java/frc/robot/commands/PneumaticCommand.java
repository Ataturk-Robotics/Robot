// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class PneumaticCommand extends CommandBase {

  public IntakeSubsystem intakeSubsystem;

  /** Creates a new SolenoidCommand. */
  public PneumaticCommand(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intakeSubsystem.getIntakeValue() == Value.kForward){
      intakeSubsystem.closeIntake();
    }else{
      intakeSubsystem.openIntake();
    }
  }

  private boolean end = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    end = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}

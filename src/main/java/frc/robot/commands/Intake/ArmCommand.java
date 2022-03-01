// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class ArmCommand extends CommandBase {

  private IntakeSubsystem mIntakeSubsystem;

  /** Creates a new ArmCommand. */
  public ArmCommand(IntakeSubsystem intakeSubsystem) {
    mIntakeSubsystem = intakeSubsystem;
    addRequirements(mIntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(mIntakeSubsystem.getArmValue() == Value.kForward)
      mIntakeSubsystem.liftArm();
    else
      mIntakeSubsystem.dropArm();
  }
  private boolean end = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    end = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}

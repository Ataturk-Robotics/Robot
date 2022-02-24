// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Intake.ArmCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.RollerCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    var armButton = new JoystickButton(Constants.controller, Constants.kArmButton);
    var intakeButton = new JoystickButton(Constants.controller, Constants.kIntakeButton);
    var shooterButton = new JoystickButton(Constants.controller, Constants.kShooterButton);
    var alignButton = new JoystickButton(Constants.controller, Constants.kAlignButton);

    armButton.whenPressed(new ArmCommand(intakeSubsystem));
    intakeButton.whenHeld(new IntakeCommand(intakeSubsystem));
    shooterButton.whenHeld(new ShooterCommand(shooterSubsystem));
    alignButton.whenPressed(new SequentialCommandGroup(
        new RunCommand(() -> intakeSubsystem.setIntake(-0.3)).withTimeout(0.3),
        new RunCommand(() -> intakeSubsystem.setIntake(0)).withTimeout(0.1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // TODO: Autonomus Command
    return null;
  }
}

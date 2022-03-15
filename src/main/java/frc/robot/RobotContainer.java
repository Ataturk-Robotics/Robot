// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Climber.AngleCommand;
import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Intake.ArmCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
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
  ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  AngleSubsystem angleSubsystem = new AngleSubsystem();

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    var armButton = new JoystickButton(Constants.controller, Constants.kArmButton);
    var intakeButton = new JoystickButton(Constants.controller, Constants.kIntakeButton);
    var shooterButton = new JoystickButton(Constants.controller, Constants.kShooterButton);
    var alignButton = new JoystickButton(Constants.controller, Constants.kAlignButton);
    var climberButton = new JoystickButton(Constants.controller, Constants.kClimberButton);

    var topPOV = new POVButton(Constants.controller, 0);
    var bottomPOV = new POVButton(Constants.controller, 180);

    armButton.whenPressed(new ArmCommand(intakeSubsystem));
    intakeButton.whenHeld(new IntakeCommand(intakeSubsystem));
    shooterButton.whenHeld(new ShooterCommand(shooterSubsystem));
    alignButton.whenPressed(new SequentialCommandGroup(
        new RunCommand(() -> intakeSubsystem.setIntake(-0.3)).withTimeout(0.3),
        new RunCommand(() -> intakeSubsystem.setIntake(0)).withTimeout(0.1)));
    climberButton.toggleWhenPressed(new ClimberCommand(climberSubsystem));

    topPOV.whenHeld(new AngleCommand(angleSubsystem, 1));
    bottomPOV.whenHeld(new AngleCommand(angleSubsystem, -1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new RunCommand(() -> shooterSubsystem.setShooter(-0.7)).withTimeout(3),
        new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(1),
        new RunCommand(() -> intakeSubsystem.setIntake(0)).withTimeout(0.1),
        new RunCommand(() -> intakeSubsystem.setRoller(-0.5)).withTimeout(0.1),
        new DriveCommand(0.5, driveSubsystem).withTimeout(1),
        new WaitCommand(1.5),
        new DriveCommand(-0.5, driveSubsystem).withTimeout(1),
        new WaitCommand(1.5),
        new RunCommand(() -> intakeSubsystem.setRoller(0)).withTimeout(0.1),
        new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(1),
        new RunCommand(() -> intakeSubsystem.setIntake(0)).withTimeout(1),
        new RunCommand(() -> shooterSubsystem.setShooter(0)).withTimeout(1));
  }
}

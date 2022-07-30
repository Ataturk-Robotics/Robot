// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Autonomous.AutonomousCommands.FiveAutoBlue3;
import frc.robot.commands.Autonomous.Trajectorys.FiveAutoBlue3Trajectory;
import frc.robot.commands.Climber.AngleCommand;
import frc.robot.commands.Climber.ClimberCommand;
import frc.robot.commands.Intake.ArmCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.RobotSubsystems.AngleSubsystem;
import frc.robot.subsystems.RobotSubsystems.ClimberSubsystem;
import frc.robot.subsystems.RobotSubsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotSubsystems.ShooterSubsystem;

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

  FiveAutoBlue3Trajectory fiveAutoBluePaths = new FiveAutoBlue3Trajectory(driveSubsystem);

  static SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    // Configure the button bindingsro
    SmartDashboard.putData("autoChooser", autoChooser);

    autoChooser.setDefaultOption("ChooseAutonomous", null);
    autoChooser.addOption("fiveAutoBlue3", new FiveAutoBlue3(driveSubsystem, shooterSubsystem, intakeSubsystem, fiveAutoBluePaths));

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
    return autoChooser.getSelected();
  }
  
}

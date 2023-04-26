// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PneumaticCommand;
<<<<<<< HEAD
import frc.robot.subsystems.LinearActuatorSubsystem;
=======
>>>>>>> 1c5d11568cc896639cc2efe085c487831c0eee4b
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LinearActuatorSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  DriveSubsystem driveSubsystem = new DriveSubsystem();
  LinearActuatorSubsystem angleSubsystem = new LinearActuatorSubsystem();
  TurretSubsystem turretSubsystem = new TurretSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    var intakeButton = new JoystickButton(JoystickConstants.ps4Controller, PS4Controller.Button.kCross.value);
    var outIntakebutton = new JoystickButton(JoystickConstants.ps4Controller, PS4Controller.Button.kCircle.value);
    var pneumaticButton = new JoystickButton(JoystickConstants.ps4Controller, PS4Controller.Button.kTriangle.value);
    
    intakeButton.whenHeld(new IntakeCommand(intakeSubsystem, 1));
    outIntakebutton.whenHeld(new IntakeCommand(intakeSubsystem, -1));
    pneumaticButton.whenPressed(new PneumaticCommand(intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
      new RunCommand(() -> driveSubsystem.tankDrive(1, 1))
    );
  }
}

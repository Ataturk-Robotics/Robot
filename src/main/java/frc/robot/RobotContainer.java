// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.Autonomous.Autodeneme;
import frc.robot.commands.Climb.ClimbCommand;
import frc.robot.commands.Climb.GrabberCommand;
import frc.robot.commands.Intake.IntakeAngleCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DifferentialDriveSubsystem driveSubsystem = new DifferentialDriveSubsystem();
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  XboxController xboxController = JoystickConstants.xBoxController;


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    var climbingUpButton = new JoystickButton(xboxController, JoystickConstants.B_BUTTON);
    var ClimbingDownButton = new JoystickButton(xboxController, JoystickConstants.A_BUTTON);

    var GrabberUpButton = new JoystickButton(xboxController, JoystickConstants.Y_BUTTON);
    var GrabberDownButton = new JoystickButton(xboxController, JoystickConstants.X_BUTTON);

    var AngleUpButton = new JoystickButton(xboxController, JoystickConstants.RIGHT_BUMPER);
    var AngleDownButton = new JoystickButton(xboxController, JoystickConstants.LEFT_BUMPER);

    climbingUpButton.whileTrue(new ClimbCommand(climbSubsystem, 0.6));
    ClimbingDownButton.whileTrue(new ClimbCommand(climbSubsystem, -0.6));

    GrabberUpButton.whileTrue(new GrabberCommand(climbSubsystem, 0.5));
    GrabberDownButton.whileTrue(new GrabberCommand(climbSubsystem, -0.5));

    AngleUpButton.whileTrue(new IntakeAngleCommand(intakeSubsystem, -0.2));
    AngleDownButton.whileTrue(new IntakeAngleCommand(intakeSubsystem, 0.1));


    //POV BUTTON
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Autodeneme(driveSubsystem, intakeSubsystem);
  }
}

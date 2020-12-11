/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public XboxController driverController = new XboxController(0);
  

  public JoystickButton asoshlarBaslaButton = new JoystickButton(driverController, 11);
  public JoystickButton asoshlarBitirButton = new JoystickButton(driverController, 12);

  public JoystickButton takoshlarBaslaButton = new JoystickButton(driverController, 9);
  public JoystickButton takoshlarBitirButton = new JoystickButton(driverController, 10);

  public JoystickButton topuslarButton = new JoystickButton(driverController, 5);
  public JoystickButton makaroslarButton = new JoystickButton(driverController, 6);
  public JoystickButton firlatmaButton = new JoystickButton(driverController, 1);

  public JoystickButton compressorBaslaButton = new JoystickButton(driverController, 7);
  public JoystickButton compressorBitirButton = new JoystickButton(driverController, 8);
  public JoystickButton compressorIleriButton = new JoystickButton(driverController, 3);
  public JoystickButton compressorGeriButton = new JoystickButton(driverController, 4);

  public double getDriverRawAxis(int axis) {
		return this.driverController.getRawAxis(axis);
	}
  

 
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}

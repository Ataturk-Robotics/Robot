// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Joystick
  public static final class JoystickConstants{
    static int  kXboxControllerPort = 0;
    public static XboxController xBoxController = new XboxController(kXboxControllerPort);
    public static int xBoxXAxis = 0;
    public static int xBoxyAxis = 1;
    public static int xBoxpowerAxis = 3;
    public static final int RIGHT_X_AXIS = XboxController.Axis.kRightX.value;
    public static final int RIGHT_Y_AXIS = XboxController.Axis.kRightY.value;
    public static final int LEFT_X_AXIS = XboxController.Axis.kLeftX.value;
    public static final int LEFT_Y_AXIS = XboxController.Axis.kLeftY.value;
    
    // Xbox Buttons
    public static final int A_BUTTON = XboxController.Button.kA.value;
    public static final int B_BUTTON = XboxController.Button.kB.value;
    public static final int X_BUTTON = XboxController.Button.kX.value;
    public static final int Y_BUTTON = XboxController.Button.kY.value;
    public static final int LEFT_BUMPER = XboxController.Button.kLeftBumper.value;
    public static final int RIGHT_BUMPER = XboxController.Button.kRightBumper.value;
    
    // Xbox Triggers
    public static final int LEFT_TRIGGER = XboxController.Axis.kLeftTrigger.value;
    public static final int RIGHT_TRIGGER = XboxController.Axis.kRightTrigger.value;
    

    // * PS4
    private static int kPs4ControllerPort = 1;
    public static PS4Controller ps4Controller = new PS4Controller(kPs4ControllerPort);
    public static int Ps4XAxis = 0;
    public static int Ps4yAxis = 1;
    public static int ps4ZAxis = 5;
  }

  //MotorIDs
  public static final class MotorIDConstants{
    //Climbing
    public static final int ClimbingMotorID = 12;
    public static final int GrabberMotorID = 13;

    //Intake
    public static final int IntakeNeoMotorID = 11;
    public static final int IntakeCimMotorID = 10;
    public static final int IntakeAngleMotorID = 3;

    //Elevator
    public static final int[] ElevatorMotorID = {8, 9};

    //Drive
    public static int[] leftMotorIds = {5, 4};
    public static int[] rightMotorIds = {7, 2};


  }
}

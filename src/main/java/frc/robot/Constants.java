// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // The controller itself
    private static int kXboxControllerPort = 0;    
    public static XboxController controller = new XboxController(kXboxControllerPort);

    // Axes
    public static int xAxis = 0;
    public static int yAxis = 1;
    public static int powerAxis = 2;

    // Buttons on the controller
    public static int kIntakeButton = 2;
    public static int kArmButton = 3;
    
    //The double solenoid used to control the pneumatic arm
    public static int[] kIntakeSolenoidPorts = { 0, 1 };

    //Motor Id's
    public static int rollerMotorId = 1;
    public static int intakeMotorId = 2;

    public static int kRightMotorId = 3;
    public static int kLeftMotorId = 4;
}
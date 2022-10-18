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
    public static int powerAxis = 3;

    // Buttons on the controller
    public static int kIntakeButton = 3;
    public static int kArmButton = 2;
    public static int kShooterButton = 1;
    public static int kAlignButton = 7;
    public static int kClimberButton = 10;
    public static int kAimButton = 4;

    public static double kAimP = 1;
    
    //The double solenoid used to control the pneumatic arm
    public static int[] kIntakeSolenoidPorts = { 2, 4 };

    public static double kCenterX;
    public static double kTargetFound = 0;

    //Motor Id's
    public static int rollerMotorId = 13;
    public static int intakeMotorId = 17;
    public static int[] shooterMotorIds = {15, 16};

    public static int kRightMotorId = 11;
    public static int kLeftMotorId = 10;

    public static int[] climberMotorIds = {18, 19};
}

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

    public static final class JoystickConstants{
        // * Xbox
        private static int kXboxControllerPort = 0;
        public static XboxController xBoxController = new XboxController(kXboxControllerPort);
        public static int xBoxXAxis = 1;
        public static int xBoxyAxis = 0;
        public static int xBoxpowerAxis = 3;

        // * PS4
        private static int kPs4ControllerPort = 1;
        public static PS4Controller ps4Controller = new PS4Controller(kPs4ControllerPort);
        public static int Ps4XAxis = 0;
        public static int Ps4yAxis = 1;
        public static int ps4ZAxis = 5;
    }
    

    public static final class DriveConstants{
        // Drive Motors
        public static int[] leftMotorIds = {1, 0};
        public static int[] rightMotorIds = {3, 4};

        public static final boolean kMotorsInverted = true;

        // * Encoders
        public static final int[] kLeftEncoderPorts = {6, 7};
        public static boolean kLeftEncoderReversed = true;

        public static final int[] kRightEncoderPorts = {8, 9};
        public static boolean kRightEncoderReversed = true;

        public static final double kEncoderDistancePerPulse = 1.0/4.0; 
    }

    public static final class SubsystemConstants{
        public static int[] angleMotorIds = {4, 6};
        public static int turretMotorId = 2;
        public static int[] armMotorIds = {5, 7};
        public static int intakeMotorId = 8;
    }

    
    
    
    

    

    
}

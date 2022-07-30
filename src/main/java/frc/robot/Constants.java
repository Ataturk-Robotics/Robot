// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

    public static boolean tuningMode = false;
    public static final RobotType defaultRobot = RobotType.ROBOT_2022;
    private static RobotType robot;

    public static final boolean flatTarget = false;

    public static final double fieldLength = 52 * 12 + 5.25;
    public static final double fieldWidth = 26 * 12 + 11.25;
    public static final double initiationLine = 120;
    public static final double visionTargetHorizDist = 43.75 + 24;
    public static final double innerPortDepth = 29.26;
    public static final double trenchRunWidth = (4 * 12) + 7.5;

    public static final int[] kLeftEncoderPorts = new int[] {4, 5};
    public static final int[] kRightEncoderPorts = new int[] {2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kPDriveVel = 8;


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
    
    //The double solenoid used to control the pneumatic arm
    public static int[] kIntakeSolenoidPorts = { 2, 4 };

    //Motor Id's
    public static int rollerMotorId = 13;
    public static int intakeMotorId = 17;
    public static int[] shooterMotorIds = {15, 16};

    public static int kRightMotorId = 11;
    public static int kLeftMotorId = 10;

    public static int[] climberMotorIds = {18, 19};
    
    public static double fieldLenght;

    public static RobotType getRobot() {
        return robot;
    }

    public static void setRobot(RobotType robot) {
        if (Constants.robot == null) {
            Constants.robot = robot;
        }
    }

    public enum RobotType {
        ROBOT_2022
    }
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  Thread m_VisionThread;

  private NetworkTable vision;
  private NetworkTableEntry xEntry;
  private NetworkTableEntry isTargetFound;


  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
    comp.disable();

    m_robotContainer = new RobotContainer();

    m_VisionThread = new Thread(
        () -> {
          UsbCamera camera = CameraServer.startAutomaticCapture();
          camera.setResolution(1080, 720);

          CvSink cvSink = CameraServer.getVideo();
          CvSource outputStream = CameraServer.putVideo("Target", 640, 480);

          Mat mat = new Mat();

          final int[] targetBoxSize = { 100, 100 };

          int targetOffset = 0;

          while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
              outputStream.notifyError(cvSink.getError());
              continue;
            }
            targetOffset = (int) (Constants.controller.getRawAxis(Constants.powerAxis) * 100);
            Imgproc.rectangle(mat,
                new Point(
                    (outputStream.getVideoMode().width - targetBoxSize[0]) / 2,
                    (outputStream.getVideoMode().height - targetBoxSize[1]) / 2 + targetOffset),
                new Point(
                    (outputStream.getVideoMode().width + targetBoxSize[0]) / 2,
                    (outputStream.getVideoMode().height + targetBoxSize[1]) / 2 + targetOffset),
                new Scalar(255, 100, 100),
                2);
            outputStream.putFrame(mat);
          }
        });
    m_VisionThread.setDaemon(true);
    m_VisionThread.start();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    vision = inst.getTable("/vision");
    xEntry = vision.getEntry("CenterX");
    isTargetFound = vision.getEntry("isTargetFound");

    // Shuffleboard.getTab("LiveWindow").add("Pi", cV3);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Constants.kCenterX = xEntry.getDouble(0);
    Constants.kTargetFound = isTargetFound.getDouble(0);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


/**
 * An example command.  You can replace me with your own command.
 */
public class C_Goruntu extends Command {
  public C_Goruntu() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.sGoruntu);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Filesystem.getDeployDirectory().getPath()
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
    MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
    mjpegServer1.setSource(usbCamera);
    CvSink cvSink = new CvSink("opencv_USB Camera 0");
    cvSink.setSource(usbCamera);
    CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
    MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
    mjpegServer2.setSource(outputStream);
    // System.out.println("C_Goruntu.execute()");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

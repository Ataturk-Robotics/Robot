// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class Aim extends CommandBase {
  /** Creates a new Aim. */
  DriveSubsystem mDriveSubsystem;
  double mIsTargetFound;
  double mCenterPoint;
  

  public Aim(DriveSubsystem driveSubsystem, double isTargetFound, double centerPoint) {
    mDriveSubsystem = driveSubsystem;
    mIsTargetFound = isTargetFound;
    mCenterPoint = centerPoint;
    

    addRequirements(mDriveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mIsTargetFound == 1.) {
      double step = mCenterPoint - 160;
      double error = step / 160;
      System.out.println(error + ", " + error * Constants.kAimP);
      mDriveSubsystem.setLeftMotor(error * Constants.kAimP);
      mDriveSubsystem.setRightMotor(-error * Constants.kAimP);
    } 
    else if (mIsTargetFound == 0.) {
      System.out.println("Target not found.");
    } 
    else {
      // Javada double karşılaştırmasını bilmiyorum
      System.out.println("Internal vision double error");
    }
  }
}

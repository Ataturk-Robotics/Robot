// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.Trajectorys.FiveAutoBlue3Trajectory;
import frc.robot.subsystems.RobotSubsystems.DriveSubsystem;
import frc.robot.subsystems.RobotSubsystems.IntakeSubsystem;
import frc.robot.subsystems.RobotSubsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveAutoBlue3 extends SequentialCommandGroup {
  /** Creates a new FiveAutoBlue3. */
  public FiveAutoBlue3(
    DriveSubsystem driveSubsystem,
    ShooterSubsystem shooterSubsystem,
    IntakeSubsystem intakeSubsystem,
    FiveAutoBlue3Trajectory paths
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new RunCommand(() -> shooterSubsystem.setShooter(-0.7)).withTimeout(1.0),
      new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(0.3).andThen(() -> intakeSubsystem.setIntake(0)),    
      new RunCommand(() -> shooterSubsystem.setShooter(0)).withTimeout(0.1),

      paths.firstBallRamsete().andThen(() -> driveSubsystem.tankDriveVolts(0, 0)),
      new RunCommand(() -> intakeSubsystem.setIntake(-0.3)).withTimeout(0.3).andThen(() -> intakeSubsystem.setIntake(0)),
      paths.secondBallRamsete().andThen(() -> driveSubsystem.tankDriveVolts(0, 0)),
      new RunCommand(() -> intakeSubsystem.setIntake(-0.3)).withTimeout(0.3).andThen(() -> intakeSubsystem.setIntake(0)),
      
      paths.throwingBallRamsete().andThen(() -> driveSubsystem.tankDriveVolts(0, 0)),

      new RunCommand(() -> shooterSubsystem.setShooter(-0.7)).withTimeout(1.0),
      new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(0.2).andThen(() -> intakeSubsystem.setIntake(0)),
      new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(0.2).andThen(() -> intakeSubsystem.setIntake(0)),
      new RunCommand(() -> shooterSubsystem.setShooter(0)).withTimeout(0.1),

      paths.thirdBallRamsete().andThen(() -> driveSubsystem.tankDriveVolts(0, 0)),
      new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(1).andThen(() -> intakeSubsystem.setIntake(0)),
      paths.fourthBallRamsete().andThen(() -> driveSubsystem.tankDriveVolts(0, 0)),

      new RunCommand(() -> shooterSubsystem.setShooter(-0.7)).withTimeout(1.0),
      new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(0.2).andThen(() -> intakeSubsystem.setIntake(0)),
      new RunCommand(() -> intakeSubsystem.setIntake(0.5)).withTimeout(0.2).andThen(() -> intakeSubsystem.setIntake(0)),
      new RunCommand(() -> shooterSubsystem.setShooter(0)).withTimeout(0.1)
    );
    driveSubsystem.resetOdometry(paths.allPathTrajectory().getInitialPose());
  }

  /* public double getFirstRobotPoseX(){
    return 7.903;
  }

  public double getFirstRobotPoseY(){
    return 2.421000;
  }

  public Rotation2d getFirstRobotPoseRotation(){
    return new Rotation2d(-111.347106);
  } */

}

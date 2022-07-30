package frc.robot.commands.Autonomous.Trajectorys;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.RobotSubsystems.DriveSubsystem;

public class FiveAutoBlue3Trajectory{
    
    String allPathTrajectoryJson = "C:/Users/BY/Desktop/output/yol1.wpilib.json";
    String firstTrajectoryJson = "C:/Users/BY/Desktop/output/firstball.wpilib.json";
    String secondTrajectoryJson = "C:/Users/BY/Desktop/output/secondball.wpilib.json";
    String throwingTrajectoryJson = "C:/Users/BY/Desktop/output/throwingball.wpilib.json";
    String thirdTrajectoryJson = "C:/Users/BY/Desktop/output/thirdball.wpilib.json";
    String fourthTrajectoryJson = "C:/Users/BY/Desktop/output/fourthball.wpilib.json";

    Trajectory firstBallTrajectory;
    Trajectory allPathTrajectory;
    Trajectory secondBallTrajectory;
    Trajectory throwingBallTrajectory;
    Trajectory thirdBallTrajectory;
    Trajectory fourthBallTrajectory;

    DriveSubsystem driveSubsystem;

    public FiveAutoBlue3Trajectory(DriveSubsystem driveSubsystem){
        try{
            Path allTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(allPathTrajectoryJson);
            Path firstTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(firstTrajectoryJson);
            Path secondTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(secondTrajectoryJson);
            Path throwingTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(throwingTrajectoryJson);
            Path thirdTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(thirdTrajectoryJson);
            Path fourthTrajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(fourthTrajectoryJson);
            
            allPathTrajectory = TrajectoryUtil.fromPathweaverJson(allTrajectoryPath);
            firstBallTrajectory = TrajectoryUtil.fromPathweaverJson(firstTrajectoryPath);
            secondBallTrajectory = TrajectoryUtil.fromPathweaverJson(secondTrajectoryPath);
            throwingBallTrajectory = TrajectoryUtil.fromPathweaverJson(throwingTrajectoryPath);
            thirdBallTrajectory = TrajectoryUtil.fromPathweaverJson(thirdTrajectoryPath);
            fourthBallTrajectory = TrajectoryUtil.fromPathweaverJson(fourthTrajectoryPath);
      
          }catch(IOException ex){
            DriverStation.reportError("Unable to open trajectory: " + firstTrajectoryJson, ex.getStackTrace());
        }

        this.driveSubsystem = driveSubsystem;
    }

    public RamseteCommand firstBallRamsete(){
        RamseteCommand firstBallRamsete = new RamseteCommand(
        firstBallTrajectory,
            driveSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter
            ),
            Constants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem
        );

        return firstBallRamsete;
    }

    public RamseteCommand secondBallRamsete(){
        RamseteCommand secondBallRamsete = new RamseteCommand(
        secondBallTrajectory,
            driveSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter
            ),
            Constants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem
        );

        return secondBallRamsete;
    }

    public RamseteCommand throwingBallRamsete(){
        RamseteCommand throwingBallRamsete = new RamseteCommand(
        throwingBallTrajectory,
            driveSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter
            ),
            Constants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem
        );

        return throwingBallRamsete;
    }

    public RamseteCommand thirdBallRamsete(){
        RamseteCommand thirdBallRamsete = new RamseteCommand(
        thirdBallTrajectory,
            driveSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter
            ),
            Constants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem
        );

        return thirdBallRamsete;
    }

    public RamseteCommand fourthBallRamsete(){
        RamseteCommand fourthBallRamsete = new RamseteCommand(
        fourthBallTrajectory,
            driveSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter
            ),
            Constants.kDriveKinematics,
            driveSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts,
            driveSubsystem
        );

        return fourthBallRamsete;
    }

    public Trajectory allPathTrajectory(){
        return allPathTrajectory;
    }

}

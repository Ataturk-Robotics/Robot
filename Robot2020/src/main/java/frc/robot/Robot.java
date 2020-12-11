/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SS_Asilma;
import frc.robot.subsystems.SS_Goruntu;
import frc.robot.subsystems.SS_Hareket;
import frc.robot.subsystems.SS_Pneumatic;
import frc.robot.subsystems.SS_TopAlma;
import frc.robot.subsystems.SS_TopFirlatma;
import frc.robot.subsystems.SS_TopTasima;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;

  Command m_autonomousCommand;
  public static SS_Hareket sHareket = new SS_Hareket();
  public static SS_Asilma sAsilma = new SS_Asilma();
  public static SS_TopAlma sTopAlma = new SS_TopAlma();
  public static SS_TopTasima sTopTasima = new SS_TopTasima();
  public static SS_TopFirlatma sTopFirlatma = new SS_TopFirlatma();
  public static SS_Pneumatic sPneumatic = new SS_Pneumatic();
  public static SS_Goruntu sGoruntu = new SS_Goruntu();
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  WPI_VictorSPX sagMotor = new WPI_VictorSPX(11);
  WPI_VictorSPX solMotor = new WPI_VictorSPX(10);

  WPI_VictorSPX firlatoslarMotor = new WPI_VictorSPX(RobotMap.firlatmaMotorID);
  WPI_VictorSPX makaroslarMotor = new WPI_VictorSPX(RobotMap.topTasimaMotorID);
  Timer time = new Timer();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  Compressor com = new Compressor(0);
  @Override
  public void robotInit() {
    m_oi = new OI();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    com.stop();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_chooser.getSelected();

    time.reset();
    time.start();

    firlatoslarMotor.set(ControlMode.PercentOutput, 0.8);
    while(time.get()<3){}
    makaroslarMotor.set(ControlMode.PercentOutput, -0.8);
    time.reset();
    time.start();
    while(time.get()<0.25){}
    makaroslarMotor.set(ControlMode.PercentOutput, 0.0);
    time.reset();
    time.start();
    while(time.get()<1){}
    makaroslarMotor.set(ControlMode.PercentOutput, -0.8);
    time.reset();
    time.start();
    while(time.get()<0.25){}
    makaroslarMotor.set(ControlMode.PercentOutput, 0.0);
    time.reset();
    time.start();
    while(time.get()<1){}
    makaroslarMotor.set(ControlMode.PercentOutput, -0.8);
    while(time.get()<1){} 
    makaroslarMotor.set(ControlMode.PercentOutput, 0.0);
    firlatoslarMotor.set(ControlMode.PercentOutput, 0.0);

    sagMotor.set(ControlMode.PercentOutput, -0.4);
    solMotor.set(ControlMode.PercentOutput, 0.4);
    while(time.get()<2){}
    sagMotor.set(ControlMode.PercentOutput, 0);
    solMotor.set(ControlMode.PercentOutput, 0);
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
      **/
    
  }

  
  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //Scheduler.getInstance().run();
    
    
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

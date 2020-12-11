/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.C_Goruntu;

/**
 * An example subsystem. You can replace with me with your own subsystem.
 */
public class SS_Goruntu extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  @Override
  protected void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new C_Goruntu());
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class ShootOne extends InstantCommand {
  /**
   * Creates a new ShootOne.
   */
  public ShootOne() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Shooter);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_Shooter.shootOne();
  }


  
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.m_Shooter;

public class ShootBall extends InstantCommand {
  XboxController controller = new XboxController((Constants.DRIVER2_ID));
  public ShootBall(){
    
  }

  /**
   * Creates a new Shooter.
   */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new RunCommand(() -> {
      m_Shooter.startShooter();
      m_Shooter.startFeeder();
      System.out.println("Currently setting");
    }).withTimeout(2)
    .andThen(() -> {
      m_Shooter.stopShooter();
      m_Shooter.stopFeeder();
      System.out.println("Turning off");
    }).schedule();
  }

}

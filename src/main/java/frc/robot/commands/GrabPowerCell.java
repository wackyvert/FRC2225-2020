/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RollerIntake;

import static frc.robot.subsystems.RollerIntake.grabPowerCell;
import static frc.robot.subsystems.RollerIntake.stop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class GrabPowerCell extends CommandBase {
  XboxController b = new XboxController(2);
  public GrabPowerCell() {
    addRequirements(RobotContainer.m_RollerIntake);
  }

  // Called when the command is initially scheduled.


@Override
  public void execute(){
  grabPowerCell();
  System.out.println("bbbbbb");
}

}
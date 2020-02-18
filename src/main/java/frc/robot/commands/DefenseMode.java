/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DefenseMode extends CommandBase {
  private final VictorSPX rollerIntake = new VictorSPX(5);
  /**
   * Creates a new DefenseMode.
   */
  public DefenseMode() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_RollerIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rollerIntake.setInverted(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollerIntake.set(ControlMode.PercentOutput, Constants.OPTIMAL_INTAKE_SPEED);
  }
}

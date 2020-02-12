/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AlignForwardAndSide extends CommandBase {
  /**
   * Creates a new AlignForwardAndSide.
   */
  public float left_command;
  public float right_command;
  public AlignForwardAndSide() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain, RobotContainer.m_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    float KpAim = -0.1f;
    float KpDistance = -0.1f;
    float min_aim_command = 0.05f;


    float tx = (float) RobotContainer.m_Limelight.getHorizontalOffset();
    float ty = (float) RobotContainer.m_Limelight.getVerticalOffset();



    float heading_error = -tx;
    float distance_error = -ty;
    float steering_adjust = 0.0f;

    if (tx > 1.0)
    {
      steering_adjust = KpAim*heading_error - min_aim_command;
    }
    else if (tx < 1.0)
    {
      steering_adjust = KpAim*heading_error + min_aim_command;
    }

    float distance_adjust = KpDistance * distance_error;

    left_command += steering_adjust + distance_adjust;
    right_command -= steering_adjust + distance_adjust;
    RobotContainer.m_Drivetrain.visionVoltage(left_command, right_command);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

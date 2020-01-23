/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.Drivetrain.*;
import edu.wpi.first.hal.sim.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * ArcadeDrive command - uses the Drivetrain subsystem. Made for FRC2225
 */
public class ArcadeDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
   

  /**
   * Creates a new ArcadeDrive command
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(Drivetrain Drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_Drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    /*
    m_Drivetrain is the instance of the Drivetrain subsystem which contains the setVoltage method. It takes parameters speedLeft and speedRight, to
    individually set the motor voltage to each side. If we set each side to the same voltage, the robot wouldn't turn.
    */

    Robot.m_Drivetrain.setVoltage(OI.calculateLeftSpeed(), OI.calculateRightSpeed());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.m_Drivetrain.setVoltage(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

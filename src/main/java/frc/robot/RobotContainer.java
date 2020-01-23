/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DirectionSwitch;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_Drivetrain = new Drivetrain();

  private final ArcadeDrive m_ArcadeDrive = new ArcadeDrive(m_Drivetrain);

  private final Joystick controller1 = new Joystick(Constants.DRIVER1_ID);
  

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Initializing every button on the controller so we can bind them to commands
    final JoystickButton aButton = new JoystickButton(controller1, 1);
    final JoystickButton bButton = new JoystickButton(controller1, 2);
    final JoystickButton xButton = new JoystickButton(controller1, 3);
    final JoystickButton yButton = new JoystickButton(controller1, 4);
    final JoystickButton rightBumperButton = new JoystickButton(controller1, 5);
    final JoystickButton leftBumperButton = new JoystickButton(controller1, 6);
    final JoystickButton squareButton = new JoystickButton(controller1, 7);
    final JoystickButton startButton = new JoystickButton(controller1, 8);
    final JoystickButton righJoystickButton = new JoystickButton(controller1, 9);
    final JoystickButton leftJoystickButton = new JoystickButton(controller1, 10);

    xButton.whenPressed(new DirectionSwitch());




  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}

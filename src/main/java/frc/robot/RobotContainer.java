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
import frc.robot.commands.AlignForwardAndSide;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DirectionSwitch;
import frc.robot.commands.GrabPowerCell;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ShootOne;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import java.awt.*;
import java.nio.file.DirectoryStream;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain m_Drivetrain = new Drivetrain();
  public static final ArcadeDrive m_ArcadeDrive = new ArcadeDrive(m_Drivetrain);
  public static final Shooter m_Shooter = new Shooter();
  public static final ShootBall m_shootBall = new ShootBall();
  public static final DirectionSwitch m_DirectionSwitch = new DirectionSwitch();
  public static final RollerIntake m_RollerIntake = new RollerIntake();
  private final Joystick controller1 = new Joystick(Constants.DRIVER1_ID);
  private final XboxController XboxController1 = new XboxController(Constants.DRIVER1_ID);
  private final Joystick controller2 = new Joystick(Constants.DRIVER2_ID);
  private GrabPowerCell bruh = new GrabPowerCell();
  public static LimelightSubsystem m_Limelight = new LimelightSubsystem();
  public static final AlignForwardAndSide m_Align = new AlignForwardAndSide();


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
    final JoystickButton aButton1 = new JoystickButton(controller1, 1);
    final JoystickButton bButton1 = new JoystickButton(controller1, 2);
    final JoystickButton xButton1 = new JoystickButton(controller1, 3);
    final JoystickButton yButton1 = new JoystickButton(controller1, 4);
    final JoystickButton aButton2 = new JoystickButton(controller2, 1);
    final JoystickButton bButton2 = new JoystickButton(controller2, 2);
    final JoystickButton xButton2 = new JoystickButton(controller2, 3);
    final JoystickButton yButton2 = new JoystickButton(controller2, 4);
    final JoystickButton rightBumperButton = new JoystickButton(controller1, 5);
    final JoystickButton leftBumperButton = new JoystickButton(controller1, 6);
    final JoystickButton squareButton = new JoystickButton(controller1, 7);
    final JoystickButton startButton = new JoystickButton(controller1, 8);
    final JoystickButton rightJoystickButton = new JoystickButton(controller1, 9);
    final JoystickButton leftJoystickButton = new JoystickButton(controller1, 10);
    final JoystickButton rightBumperButton2 = new JoystickButton(controller1, 5);
    final JoystickButton leftBumperButton2 = new JoystickButton(controller1, 6);
    final JoystickButton squareButton2 = new JoystickButton(controller1, 7);
    final JoystickButton startButton2 = new JoystickButton(controller1, 8);
    final JoystickButton rightJoystickButton2 = new JoystickButton(controller1, 9);
    final JoystickButton leftJoystickButton2 = new JoystickButton(controller1, 10);



    //xButton2.whenPressed(new ShootOne());


    xButton1.whenPressed(new DirectionSwitch());
    xButton2.whenPressed(new ShootBall());
    aButton2.whileHeld(new GrabPowerCell());
    aButton2.whenReleased(new StopIntake());
    leftJoystickButton2.whileHeld(new AlignForwardAndSide(), true);
  }
  //Getting the trigger as a button instead of an axis, as we dont really want any variability in how fast the shooter goes


  /*
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}


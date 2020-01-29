/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.DirectionSwitch;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.Map;

import static edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance;
import static frc.robot.RobotContainer.m_Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
public VictorSPX shooterMotor = new VictorSPX(5);
  public TalonSRX feeder = new TalonSRX(6);
  private NetworkTableEntry joyOrX;
  private NetworkTableEntry shooterSlider;
  private NetworkTableEntry intakeSlider;
  private RobotContainer m_robotContainer;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private double shooterVal;
  private double intakeVal;
  private final XboxController controller = new XboxController(0);
    

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
      joyOrX = Shuffleboard.getTab("Robot Control")
              .add("Joystick Enabled?", false)
              .withWidget("Toggle Button")
              .getEntry();
      //Slider to set shooter motor
      shooterSlider = Shuffleboard.getTab("Shooter")
              .add("Output Value", 0)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", -1, "max", 1))
              .getEntry();

      intakeSlider = Shuffleboard.getTab("Intake")
              .add("Output Value", 0)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", -1, "max", 1))
              .getEntry();
    //makes new camera  
    final CameraServer camera = CameraServer.getInstance();
      camera.startAutomaticCapture();
    m_robotContainer = new RobotContainer();
    //Sets the default command of the drivetrain subsystem to arcade drive
    getInstance().setDefaultCommand(RobotContainer.m_Drivetrain, RobotContainer.m_ArcadeDrive);
    //Sets the default command of the shooter subsystem to the shootBall command
    //getInstance().setDefaultCommand(RobotContainer.m_Shooter, RobotContainer.m_shootBall);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putData("Direction Switch", new DirectionSwitch());

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    
   //The scheduler that runs the commands. DONT TOUCH
    getInstance().run();
        
    //Color sensor code
    final Color detectedColor = m_colorSensor.getColor();
        String colorString;
        final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
        //Code to switch between the joystick or xbox controller for DRIVING ONLY.
        if(joyOrX.getBoolean(false))
        {
          Constants.ControllerAxisNum[0] = 0;
          Constants.ControllerAxisNum[1] = 2;
        }
        else
        {
          Constants.ControllerAxisNum[0]=4;
          Constants.ControllerAxisNum[1]=1;
        }
        intakeVal = intakeSlider.getDouble(0);
        shooterVal = intakeSlider.getDouble(0);

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }
    public void updateTracking(){

        final float Kp = -0.1f;
        final float min_command = 0.05f;

        NetworkTableInstance.getDefault().getTable("vision").getEntry("X").getDouble(0);

        NetworkTableInstance.getDefault().getTable("vision").getEntry("Y").getDouble(0);
        final float tx =(float)NetworkTableInstance.getDefault().getTable("vision").getEntry("X").getDouble(0);

        System.out.println("X: "+tx);
        if (controller.getAButton())
        {
            final float heading_error = -tx;
            float steering_adjust = 0.0f;
            if (tx > 300)
            {
                steering_adjust = Kp*heading_error - min_command;
            }
            else if (tx < 300)
            {
                steering_adjust = Kp*heading_error + min_command;
            }
            
        }}

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    

      //shooterMotor.set(ControlMode.PercentOutput, -shooterSlider.getDouble(0));
     // m_Shooter.startFeeder();
      if(controller.getYButtonPressed()){
          //feeder.set(ControlMode.PercentOutput, 1);
      }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

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
import frc.robot.commands.Shooter;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.networktables.NetworkTableEntry;

import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final Drivetrain m_Drivetrain = new Drivetrain();
  public static final Shooter m_Shooter = new Shooter();
  public static final frc.robot.subsystems.Shooter m_ShooterSystem = new frc.robot.subsystems.Shooter();
  public VictorSPX shooterMotor = new VictorSPX(5);
  public TalonSRX feeder = new TalonSRX(6);
  private NetworkTableEntry joyOrX;
  public static NetworkTableEntry shooterSlider;
  public static NetworkTableEntry intakeSlider;
  public static final ArcadeDrive m_ArcadeDrive = new ArcadeDrive(m_Drivetrain);
//This is code for the color sensor we use for the wheel of fortune
  private RobotContainer m_robotContainer;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private double shooterVal;
  private double intakeVal;
  private XboxController controller = new XboxController(Constants.JOYSTICK_ID);
    double left_command;
    double right_command;

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
private double driveCommand;
private double steerCommand;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
      CameraServer camera = CameraServer.getInstance();
      camera.startAutomaticCapture();
    //This is a toggle button that shows up on the dashboard so I can switch between using a joystick or a controller
    SmartDashboard.putData("Direction Switch", new DirectionSwitch());
    joyOrX = Shuffleboard.getTab("Robot Control")
                .add("Joystick Enabled?", false)
                .withWidget("Toggle Button")
                .getEntry();
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
    m_robotContainer = new RobotContainer();
    CommandScheduler.getInstance().setDefaultCommand(m_Drivetrain, m_ArcadeDrive);
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    
   //The scheduler that runs the commands. DONT TOUCH
    CommandScheduler.getInstance().run();
        
    //Color sensor code
    Color detectedColor = m_colorSensor.getColor();
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

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

        float Kp = -0.1f;
        float min_command = 0.05f;

        NetworkTableInstance.getDefault().getTable("vision").getEntry("X").getDouble(0);

        NetworkTableInstance.getDefault().getTable("vision").getEntry("Y").getDouble(0);
        float tx =(float)NetworkTableInstance.getDefault().getTable("vision").getEntry("X").getDouble(0);

        System.out.println("X: "+tx);
        if (controller.getAButton())
        {
            float heading_error = -tx;
            float steering_adjust = 0.0f;
            if (tx > 300)
            {
                steering_adjust = Kp*heading_error - min_command;
            }
            else if (tx < 300)
            {
                steering_adjust = Kp*heading_error + min_command;
            }
            left_command += steering_adjust;
            right_command -= steering_adjust;
        }}

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
      shooterMotor.set(ControlMode.PercentOutput, controller.getTriggerAxis(GenericHID.Hand.kRight));
      feeder.set(ControlMode.PercentOutput, controller.getTriggerAxis(GenericHID.Hand.kLeft));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}

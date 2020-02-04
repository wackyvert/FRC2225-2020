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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.util.Map;

import static edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance;

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
  private final XboxController controller = new XboxController(1);
    

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
      //Changing between xbox controller or joystick
      joyOrX = Shuffleboard.getTab("Robot Control")
              .add("Joystick Enabled?", false)
              .withWidget("Toggle Button")
              .getEntry();
      /*Slider to set shooter motor - deprecated because we have the command for it now
      shooterSlider = Shuffleboard.getTab("Shooter")
              .add("Output Value", 0)
              .withWidget(B22uiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", -1, "max", 1))
              .getEntry();

       */

         // Slider to set intake motor once its mounted and working
      intakeSlider = Shuffleboard.getTab("Intake")
              .add("Output Value", 0)
              .withWidget(BuiltInWidgets.kNumberSlider)
              .withProperties(Map.of("min", -1, "max", 1))
              .getEntry();
    //makes new camera  
    final CameraServer camera = CameraServer.getInstance();
      camera.startAutomaticCapture();
    m_robotContainer = new RobotContainer();
    //Sets the default command of the drivetrain subsystem to arcade drive so I don't have to have anything in teleopPeriodic
    getInstance().setDefaultCommand(RobotContainer.m_Drivetrain, RobotContainer.m_ArcadeDrive);

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
feeder.set(ControlMode.PercentOutput, controller.getRawAxis(2));
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

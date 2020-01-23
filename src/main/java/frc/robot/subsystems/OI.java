/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ScaleInputs;

public class OI extends SubsystemBase {
  private static XboxController controller1 = new XboxController(Constants.DRIVER1_ID);
  private static Joystick sidewinder = new Joystick(Constants.JOYSTICK_ID);
  /**
   * Creates a new OI.
   */
  public OI() {

  }
  public static double getSpeedVal(){
    return ScaleInputs.scaleInputs(controller1.getRawAxis(4));
  }
  public static double getTurnVal(){
    return ScaleInputs.scaleInputs(controller1.getRawAxis(1));
  }
  public static double calculateLeftSpeed(){
    double left = getSpeedVal()-getTurnVal(); //left is speed minus turn for arcade drive
    return left;
  }
  public static double calculateRightSpeed(){ //right is speed plus turn for arcade drive
    double right = getSpeedVal()+getTurnVal();
    return right;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

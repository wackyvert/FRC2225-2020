/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final TalonSRX fl = new TalonSRX(Constants.FRONT_LEFT_ID);
  private final TalonSRX bl = new TalonSRX(Constants.BACK_LEFT_ID);
  private final TalonSRX fr = new TalonSRX(Constants.FRONT_RIGHT_ID);
  private final TalonSRX br = new TalonSRX(Constants.BACK_RIGHT_ID);
  public static boolean switcher;
  /**
   * Creates a new Drivetrain constructor
   */
  public Drivetrain() {

  }
  //Switches which side of the robot is considered "forward"
  public void switchDirection(){
    switcher^=true;
    fl.setInverted(switcher);
    bl.setInverted(switcher);
    fr.setInverted(switcher);
    br.setInverted(switcher);
  }
  //Sets the motor voltage so we can drive
  public void setVoltage(double speedLeft, double speedRight){
    fl.set(ControlMode.PercentOutput, speedLeft);
    bl.set(ControlMode.PercentOutput, speedLeft);
    fr.set(ControlMode.PercentOutput, speedRight);
    br.set(ControlMode.PercentOutput, speedRight);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, but we dont use this part because we have a command for it instead
  }
}

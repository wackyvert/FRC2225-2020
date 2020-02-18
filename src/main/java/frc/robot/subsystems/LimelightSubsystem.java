/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LimelightSubsystem extends SubsystemBase {
  NetworkTable table = NetworkTable.getTable("limelight");
double targetOffsetAngle_Horizontal; //the degrees the target is offset horizontally
double targetOffsetAngle_Vertical;  //the degrees the target is offset vertically
double targetArea; //the target area
double targetSkew; //the skew away from the target
  /**
   * Creates a new LimelightSubsystem.
   */
  public LimelightSubsystem() {
    
  }

  @Override
  public void periodic() {
    targetOffsetAngle_Horizontal= table.getNumber("tx", 0); //updates the number
    targetOffsetAngle_Vertical= table.getNumber("ty", 0); //updates the number
    targetArea = table.getNumber("ta", 0); //updates the number
    targetSkew = table.getNumber("ts", 0); //updates the number
  }
  public double getHorizontalOffset(){
    return targetOffsetAngle_Horizontal;
  }
  public double getVerticalOffset(){
    return targetOffsetAngle_Vertical;
  }
  public double getTargetArea(){
    return targetArea;
  }
  public double getTargetSkew(){
    return targetSkew;
  }
}

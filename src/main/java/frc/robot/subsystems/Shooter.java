/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
    

public class Shooter extends SubsystemBase {
 
  private static VictorSPX shooterMotor = new VictorSPX(7);
  private static TalonSRX feeder = new TalonSRX(6);
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    feeder.setInverted(true);
    

  }
  public void startFeeder(){
    feeder.set(ControlMode.PercentOutput, 1);
  }
  //method to shoot a singular ball
  public void shootOne(){
    startShooter();
    startFeeder();
    stopShooter();
    stopFeeder();
  }

  public void startShooter() {
    shooterMotor.set(ControlMode.PercentOutput, 1);
  }

  //prepare to shoot, called once at the start of the match
  public void prepareToShoot(){
    shooterMotor.set(ControlMode.PercentOutput, Constants.OPTIMAL_SHOOTER_SPEED);
  }
  //stop shooter, used when the robot is disabled or in a seperate condition if the shooter needs to be stopped
  public void stopShooter(){
    shooterMotor.set(ControlMode.PercentOutput, 0);
  }
  //runs the feeder motor so balls can be shot as long as the trigger is pushed in on the secondary trigger
  public void shootTrigger(){
    feeder.set(ControlMode.PercentOutput, 1);
    System.out.println("Work");
  }

  public void stopFeeder(){
    feeder.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

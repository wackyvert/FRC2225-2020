/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FRONT_LEFT_ID=0;  //front left motor CAN ID
    public static final int BACK_LEFT_ID=1;   //back left motor CAN ID
    public static final int FRONT_RIGHT_ID=2; //front right motor CAN ID
    public static final int BACK_RIGHT_ID=3;  //back right motor CAN ID
    public static final int DRIVER1_ID=0;     //Driver XboxController USB ID
	public static final int JOYSTICK_ID = 1;  //Driver sidewinder joystick USB ID
    public static int[] ControllerAxisNum= {4, 1};
}

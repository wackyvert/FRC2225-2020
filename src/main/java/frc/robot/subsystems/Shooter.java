package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static edu.wpi.first.wpilibj.GenericHID.Hand.kLeft;
import static edu.wpi.first.wpilibj.GenericHID.Hand.kRight;
import static frc.robot.Robot.shooterSlider;

public class Shooter extends SubsystemBase {

    private static VictorSPX shooterMotor = new VictorSPX(5);
    private static TalonSRX feederMotor = new TalonSRX(6);
    private static XboxController controller = new XboxController(Constants.JOYSTICK_ID);


    public Shooter(){

    }
    public void setShooterMotor(){
        shooterMotor.set(ControlMode.PercentOutput, -shooterSlider.getDouble(0));
        feederMotor.set(ControlMode.PercentOutput, controller.getRawAxis(3));
    }
    public static void kyleCantFigureOutShuffleboard(){

        shooterMotor.set(ControlMode.PercentOutput, controller.getTriggerAxis(kRight));
        feederMotor.set(ControlMode.PercentOutput, controller.getTriggerAxis(kLeft));
    }
}

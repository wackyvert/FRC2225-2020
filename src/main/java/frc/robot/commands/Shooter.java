package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Shooter extends CommandBase {
    public Shooter(){
        addRequirements(Robot.m_ShooterSystem);
    }
    public void execute(){
        frc.robot.subsystems.Shooter.kyleCantFigureOutShuffleboard();
    }
}

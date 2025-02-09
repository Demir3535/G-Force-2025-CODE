package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.wrist.WristSubsystem;
import edu.wpi.first.wpilibj.PS5Controller;

public class MoveWristManual extends Command {
     private PS5Controller operatorJoystick;
    WristSubsystem wristSubsystem;

    public MoveWristManual(WristSubsystem wristSubsystem,  PS5Controller operatorJoystick) {
        this.operatorJoystick = operatorJoystick;
        this.wristSubsystem = wristSubsystem;
        addRequirements(wristSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.moveAtSpeed(0);
    }

    @Override
    public void execute() {
        double speed = operatorJoystick.getRightX();    
    }

    @Override
    public void initialize() {
        wristSubsystem.moveAtSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
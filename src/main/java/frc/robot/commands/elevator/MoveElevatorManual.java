package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.wpilibj.PS4Controller; 
import edu.wpi.first.wpilibj.PS5Controller;

public class MoveElevatorManual extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private PS4Controller operatorJoystick;

    public MoveElevatorManual(ElevatorSubsystem elevatorSubsystem, PS4Controller operatorJoystick) {
        this.operatorJoystick = operatorJoystick;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        double speed = operatorJoystick.getLeftY();
    }

    @Override
    public void initialize() {
        elevatorSubsystem.moveAtSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
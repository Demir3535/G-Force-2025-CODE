package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;;
public class SetElevatorSpeed extends Command {
    ElevatorSubsystem elevatorSubsystem;
    double speed;

    public SetElevatorSpeed (ElevatorSubsystem elevatorSubsystem, double speed) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.moveAtSpeed(0);
    }

    @Override
    public void execute() {
        elevatorSubsystem.moveAtSpeed(speed);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.moveAtSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
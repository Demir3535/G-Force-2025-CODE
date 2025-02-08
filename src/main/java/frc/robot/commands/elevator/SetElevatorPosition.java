package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.RobotConstants.PortConstants.Controller;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class SetElevatorPosition extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double targetPosition;

    public SetElevatorPosition(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atPosition();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stop();
    }
}
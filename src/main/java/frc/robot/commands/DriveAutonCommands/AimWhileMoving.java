package frc.robot.commands.DriveAutonCommands;
import frc.robot.subsystems.drive.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class AimWhileMoving extends Command {
    private final DriveSubsystem m_robotDrive;
    
    // Constructor
    public AimWhileMoving(DriveSubsystem subsystem) {
        m_robotDrive = subsystem;
        addRequirements(m_robotDrive);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {
        if (m_robotDrive != null) {
            m_robotDrive.aimWhileMovingv2(m_robotDrive.turnPID());
        }
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
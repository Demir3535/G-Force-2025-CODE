package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.LimelightConstants;

public class LimelightDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final Joystick joystick;
    private final LimelightSubsystem limelight;
    
    public LimelightDriveCommand(DriveSubsystem drive, Joystick joystick, LimelightSubsystem limelight) {
        this.drive = drive;
        this.joystick = joystick;
        this.limelight = limelight;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        double xSpeed = -joystick.getRawAxis(3) * LimelightConstants.MAX_DRIVE_SPEED;
        double ySpeed = -joystick.getRawAxis(2) * LimelightConstants.MAX_DRIVE_SPEED;
        double rot = limelight.getSteer();
        drive.drive(ySpeed, xSpeed, rot, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
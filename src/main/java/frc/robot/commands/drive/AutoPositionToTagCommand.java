package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotConstants.LimelightConstants;

public class AutoPositionToTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final DriveSubsystem drive;
    private final int targetTagID; // if -1 its connect to any tag

    public AutoPositionToTagCommand(LimelightSubsystem limelight, DriveSubsystem drive, int targetTagID) {
        this.limelight = limelight;
        this.drive = drive;
        this.targetTagID = targetTagID; // specific id or any id for -1
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (limelight.hasTargets()) {
            double steer = limelight.getSteer();
            drive.drive(0, 0, steer * LimelightConstants.MAX_TURN_SPEED, true, true);
        }
    }

    @Override
    public boolean isFinished() {
        return limelight.hasTargets() && Math.abs(limelight.getTx()) < LimelightConstants.ACCEPTABLE_TX_ERROR;
    }
    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true, true); //Stop the robot when the command ends
    }
}
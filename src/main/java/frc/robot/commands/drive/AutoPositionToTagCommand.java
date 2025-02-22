package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

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
            int currentTagID = limelight.getTargetID();

            // If targetTagID is -1 or matches current target targetTagID
            if (targetTagID == -1 || currentTagID == targetTagID) {
                double steer = limelight.getSteer();
                drive.drive(0, 0, steer, true, true); // turn robot
            } else {
                drive.drive(0, 0, 0, true, true); // no tag, stop
            }
        } else {
            drive.drive(0, 0, 0, true, true); // if there no tag robot stop
        }
    }

    @Override
    public boolean isFinished() {
        if (targetTagID == -1) {
          // Docking status to any AprilTag
            return limelight.hasTargets() && Math.abs(limelight.getTx()) < 1.0;
        } else {
            //Docking status to a specific AprilTag ID
            return limelight.getTargetID() == targetTagID && Math.abs(limelight.getTx()) < 1.0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true, true); //Stop the robot when the command ends
    }
}
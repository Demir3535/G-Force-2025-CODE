package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoPositionToTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final DriveSubsystem drive;
    private final int targetTagID; // -1 ise herhangi bir AprilTag'e kenetlenir

    public AutoPositionToTagCommand(LimelightSubsystem limelight, DriveSubsystem drive, int targetTagID) {
        this.limelight = limelight;
        this.drive = drive;
        this.targetTagID = targetTagID; // Belirli bir ID veya -1 (herhangi bir AprilTag)
        addRequirements(drive);
    }

    @Override
    public void execute() {
        if (limelight.hasTargets()) {
            int currentTagID = limelight.getTargetID();

            // Eğer targetTagID -1 ise veya mevcut hedef targetTagID ile eşleşiyorsa
            if (targetTagID == -1 || currentTagID == targetTagID) {
                double steer = limelight.getSteer();
                drive.drive(0, 0, steer, true, true); // Robotu döndür
            } else {
                drive.drive(0, 0, 0, true, true); // Hedef bulunamadı, dur
            }
        } else {
            drive.drive(0, 0, 0, true, true); // Hedef yoksa, robotu durdur
        }
    }

    @Override
    public boolean isFinished() {
        if (targetTagID == -1) {
            // Herhangi bir AprilTag'e kenetlenme durumu
            return limelight.hasTargets() && Math.abs(limelight.getTx()) < 1.0;
        } else {
            // Belirli bir AprilTag ID'sine kenetlenme durumu
            return limelight.getTargetID() == targetTagID && Math.abs(limelight.getTx()) < 1.0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true, true); // Komut sonlandığında robotu durdur
    }
}
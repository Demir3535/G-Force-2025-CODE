package frc.robot.commands.apriltag;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class AutoPositionToTagCommand extends Command {
    private final LimelightSubsystem limelightSubsystem;
    private final int targetTagId;

    public AutoPositionToTagCommand(LimelightSubsystem limelightSubsystem, int targetTagId) {
        this.limelightSubsystem = limelightSubsystem;
        this.targetTagId = targetTagId;
        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {
        limelightSubsystem.autoPositionToTag(targetTagId);
    }

    @Override
    public boolean isFinished() {
        return true; // Komut tek seferde çalıştırılacak şekilde ayarlandı.
    }
}
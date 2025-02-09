package frc.robot.commands.apriltag;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.LimelightHelpers;

public class AutoPositionToTagCommand extends Command {
    private final LimelightSubsystem limelightSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final int targetTagId;
    private boolean isFinished = false;

    // Sabit değerler tanımlama
    private static final double ROTATION_P = 0.05;         // Dönüş için P katsayısı
    private static final double FORWARD_P = 0.05;          // İleri hareket için P katsayısı
    private static final double MAX_SPEED = 0.5;           // Maksimum hız
    private static final double MIN_SPEED = 0.05;          // Minimum hız (deadband)
    private static final double ANGLE_TOLERANCE = 1.0;     // Açı toleransı (derece)
    private static final double MIN_TARGET_AREA = 0.5;     // Minimum hedef alanı
    private static final boolean RATE_LIMIT = true;        // Hız sınırlama aktif/pasif

    public AutoPositionToTagCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem, int targetTagId) {
        this.limelightSubsystem = limelightSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.targetTagId = targetTagId;
        addRequirements(limelightSubsystem, driveSubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;
        driveSubsystem.drive(0, 0, 0, false, RATE_LIMIT);  // Başlangıçta robotu durdur
    }

    @Override
    public void execute() {
        // Limelight'ın bir hedef görüp görmediğini kontrol et
        if (!LimelightHelpers.getTV("limelight")) {
            driveSubsystem.drive(0, 0, 0, false, RATE_LIMIT);
            return;
        }

        double currentTagId = LimelightHelpers.getFiducialID("limelight");
        
        // Doğru AprilTag'i kontrol et
        if (currentTagId != targetTagId) {
            driveSubsystem.drive(0, 0, 0, false, RATE_LIMIT);
            return;
        }

        // Hedef verilerini al
        double x = LimelightHelpers.getTX("limelight");
        double y = LimelightHelpers.getTY("limelight");
        double area = LimelightHelpers.getTA("limelight");

        // Hız hesaplamaları
        double rotationSpeed = calculateSpeed(-x * ROTATION_P);
        double forwardSpeed = calculateSpeed(y * FORWARD_P);

        // Robot hareketi - field relative olmadan (false)
        driveSubsystem.drive(forwardSpeed, 0, rotationSpeed, false, RATE_LIMIT);

        // Hedef pozisyona ulaşma kontrolü
        if (isAtTarget(x, y, area)) {
            isFinished = true;
            driveSubsystem.drive(0, 0, 0, false, RATE_LIMIT);
        }
    }

    // Hız hesaplama yardımcı metodu
    private double calculateSpeed(double speed) {
        // Deadband uygula
        if (Math.abs(speed) < MIN_SPEED) {
            return 0;
        }
        // Hızı sınırla
        return MathUtil.clamp(speed, -MAX_SPEED, MAX_SPEED);
    }

    // Hedef pozisyona ulaşma kontrolü
    private boolean isAtTarget(double x, double y, double area) {
        return Math.abs(x) < ANGLE_TOLERANCE && 
               Math.abs(y) < ANGLE_TOLERANCE && 
               area > MIN_TARGET_AREA;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, RATE_LIMIT);
    }
}
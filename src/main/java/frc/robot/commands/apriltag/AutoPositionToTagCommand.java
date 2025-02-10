package frc.robot.commands.apriltag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.LimelightHelpers;

public class AutoPositionToTagCommand extends Command {
   private final LimelightSubsystem limelightSubsystem;
   private final DriveSubsystem driveSubsystem;
   private final int targetTagId;
   private boolean isFinished = false;

   // Sabit değerler - Hızlar arttırıldı ve rate limit kaldırıldı
   private static final double ROTATION_P = 0.8;     // %80 güç
   private static final double FORWARD_P = 0.8;      // %80 güç
   private static final double MAX_SPEED = 0.8;      // %80 max hız
   private static final double MIN_SPEED = 0.15;     // Minimum hız arttırıldı
   private static final double ANGLE_TOLERANCE = 1.0;
   private static final double MIN_TARGET_AREA = 0.5;
   private static final boolean RATE_LIMIT = false;  // Rate limit kapatıldı

   public AutoPositionToTagCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem, int targetTagId) {
       this.limelightSubsystem = limelightSubsystem;
       this.driveSubsystem = driveSubsystem;
       this.targetTagId = targetTagId;
       addRequirements(limelightSubsystem, driveSubsystem);
   }

   @Override
   public void initialize() {
       isFinished = false;
       driveSubsystem.drive(0, 0, 0, false, false);  // Rate limit kapalı
   }

   @Override
   public void execute() {
       if (!LimelightHelpers.getTV("limelight")) {
           return;
       }
   
       double x = LimelightHelpers.getTX("limelight");
       double y = LimelightHelpers.getTY("limelight");
       
       // Sadece dönüş hareketi için
       double rotationSpeed = -x * 0.15; // Dönüş hızını düşürdük
       double forwardSpeed = 0.1; // İleri hareketi devre dışı bıraktık
   
       // Hız sınırları
       rotationSpeed = MathUtil.clamp(rotationSpeed, 0.4, -0.4);
   
       // Motor kontrolü
       ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, 0, rotationSpeed);
       driveSubsystem.runChassisSpeeds(speeds, false);
   }
   private double calculateSpeed(double speed) {
       if (Math.abs(speed) < MIN_SPEED) {
           return 0;
       }
       return MathUtil.clamp(speed, -MAX_SPEED, MAX_SPEED);
   }

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
       driveSubsystem.drive(0, 0, 0, false, false);
   }
}
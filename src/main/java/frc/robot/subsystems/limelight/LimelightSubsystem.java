package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstans;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static java.lang.Math.round;

public class LimelightSubsystem extends SubsystemBase {
   private String limelightName = "limelight"; // LimeLight cihazının adını tutar
   public static boolean canSee; // Tag görüş durumunu belirtir
   static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); // LimeLight network tablosunu oluşturur
   private final DriveSubsystem driveSubsystem; // Sürüş alt sistemini tanımlar

     // Bu iki değerden birini kullanın (aynı anda ikisi değil):
     private static final double DESIRED_TAG_DISTANCE = 1;  // 120 cm için
     // VEYA
     // private static final double DESIRED_TAG_DISTANCE = 1.5;  // 150 cm için
     
     private static final double MIN_TAG_DISTANCE = 0.99;     // minimum güvenli mesafe
     

   public LimelightSubsystem(DriveSubsystem driveSubsystem) {
       this.driveSubsystem = driveSubsystem; // Sürüş alt sistemini başlatır
   }

   // Anlık görülen AprilTag ID'sini döndürür
   public double getTagID() {
       return LimelightHelpers.getFiducialID(RobotConstants.LLName);
   }

   // Belirli bir AprilTag'e öncelik verir
   public void setPriorityTag(int tagID) {
       LimelightHelpers.setPriorityTagID(RobotConstants.LLName, tagID);
   }

   // Robot pozisyonunun tahminini alır
   public LimelightHelpers.PoseEstimate getPoseEstimate() {
       return LimelightHelpers.getBotPoseEstimate_wpiBlue(RobotConstants.LLName);
   }

   // Tag ile hizalanma durumunu kontrol eder
   public boolean isAlignedWithTag() {
       double x = LimelightHelpers.getTX(RobotConstants.LLName);
       return Math.abs(x) < 2.0; // 2 derece toleransla hizalanma kontrolü
   }

   // Belirli bir tag'e robotu konumlandırır
   public void autoPositionToTag(int targetTagId) {
    double currentTagId = getTagID();
    if (currentTagId == targetTagId) {
        // Hedef tag görüş alanında ise konumlandırma yapar
        double tx = LimelightHelpers.getTX(RobotConstants.LLName);
        double currentDistance = getDistance();
        
        // Güvenli mesafe kontrolü
        if (currentDistance < MIN_TAG_DISTANCE) {
            // Çok yakınsa geri git
            ChassisSpeeds speeds = new ChassisSpeeds(-0.2, 0, 0);
            driveSubsystem.runChassisSpeeds(speeds, true);
            return;
        }
        
        // Mesafe bazlı hız hesaplama
        double distanceError = currentDistance - DESIRED_TAG_DISTANCE;
        double forwardSpeed = distanceError * 0.5; // Mesafeye göre hız
        double rotationSpeed = tx * 0.5; // Dönüş hızını hesaplar
        
        // Hız sınırlaması
        forwardSpeed = Math.min(Math.max(forwardSpeed, -0.5), 0.5);
        rotationSpeed = Math.min(Math.max(rotationSpeed, -0.5), 0.5);

        // Robotun hareketini kontrol eder
        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, 0, rotationSpeed);
        driveSubsystem.runChassisSpeeds(speeds, true);
        
        // SmartDashboard'a mesafe bilgilerini gönder
        SmartDashboard.putNumber("Target Distance", DESIRED_TAG_DISTANCE);
        SmartDashboard.putNumber("Current Distance", currentDistance);
        SmartDashboard.putNumber("Distance Error", distanceError);
    } else {
        System.out.println("Hedef AprilTag görüş alanında değil: " + targetTagId);
    }
}

   @Override
   public void periodic() {
       // Periyodik olarak LimeLight verilerini okur
       double x = LimelightHelpers.getTX(RobotConstants.LLName);
       double y = LimelightHelpers.getTY(RobotConstants.LLName);
       double area = LimelightHelpers.getTA(RobotConstants.LLName);

       // Tag görüş durumunu günceller
       if (area == 0) {
           canSee = false;
       } else {
           canSee = true;
       }

       // Verileri SmartDashboard'a gönderir
       SmartDashboard.putNumber("LimelightX", x);
       SmartDashboard.putNumber("LimelightY", y);
       SmartDashboard.putNumber("LimelightArea", area);
       SmartDashboard.putNumber("GetDistance", getDistance());
       SmartDashboard.putBoolean("cansee?", canSee);

       // AprilTag bilgilerini SmartDashboard'a gönderir
       SmartDashboard.putNumber("AprilTag ID", getTagID());

       // Pose tahminini alır ve SmartDashboard'a gönderir
       var poseEstimate = getPoseEstimate();
       if (poseEstimate != null) {
           SmartDashboard.putNumber("Tag Count", poseEstimate.tagCount);
           SmartDashboard.putNumber("Avg Tag Distance", poseEstimate.avgTagDist);

           // Robot pozisyon bilgilerini SmartDashboard'a gönderir
           Pose2d pose = poseEstimate.pose;
           SmartDashboard.putNumber("Robot X", pose.getX());
           SmartDashboard.putNumber("Robot Y", pose.getY());
           SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
       }
   }

   @Override
   public void simulationPeriodic() {
       // Simülasyon için boş periyodik metod
   }

   // Mesafeyi yuvarlar
   public static int roundDistance() {
       int distance = (int) round(getDistance());
       return distance;
   }

   // Hedef tag'e olan mesafeyi hesaplar
   public static double getDistance() {
       var y = LimelightHelpers.getTY(RobotConstants.LLName);
       double distance = (ShooterConstans.ApTagHeight - ShooterConstans.CamHeight) / Math.tan((ShooterConstans.CamAngle + y) * (Math.PI / 180));
       return distance;
   }

   // Tüm tag'lere sırayla konumlanmayı dener
   public void autoPositionToAllTags() {
       int[] priorityTags = {1, 2, 3, 4, 5, 6, 7, 8, 9}; // Olası tüm tag'ler
       
       for (int tagId : priorityTags) {
           if (isValidTagForPositioning(tagId)) {
               autoPositionToTag(tagId);
               break; // Sadece bir tag için konumlanır
           }
       }
   }

   // Tag'in konumlandırmaya uygunluğunu kontrol eder
   private boolean isValidTagForPositioning(int tagId) {
       double currentTagId = getTagID();
       var poseEstimate = getPoseEstimate();

       return currentTagId == tagId && 
              poseEstimate != null && 
              poseEstimate.tagCount == 1 && 
              poseEstimate.avgTagDist < 5.0; // Güvenilir mesafe kontrolü
   }
}
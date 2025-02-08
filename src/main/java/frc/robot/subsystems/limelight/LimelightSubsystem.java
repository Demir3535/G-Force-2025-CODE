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
           double tx = LimelightHelpers.getTX(RobotConstants.LLName); // Yatay açıyı alır
           double ty = LimelightHelpers.getTY(RobotConstants.LLName); // Dikey açıyı alır

           // Robot hareketini hesaplar
           double rotationSpeed = tx * 0.05; // Dönüş hızını hesaplar
           double forwardSpeed = ty * 0.05; // İleri/geri hızını hesaplar

           // Robotun hareketini kontrol eder
           ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, 0, rotationSpeed);
           driveSubsystem.runChassisSpeeds(speeds, true); // Alan bazlı hareket
       } else {
           // Hedef tag görüş alanında değilse konsola mesaj yazdırır
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
       int[] priorityTags = {1, 2, 3, 4, 5, 6, 7, 8}; // Olası tüm tag'ler
       
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
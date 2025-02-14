package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstans;
import frc.robot.RobotConstants;
import frc.robot.LimelightHelpers;
import static java.lang.Math.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants.VisionConstants;

public class Limelight extends SubsystemBase {
   // Sınıf değişkenleri
   private static double tx = 7.0;
   private static double ty = 17.1412414;
   private static double targetArea;
   public static boolean canSee;

   static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

   // tx için getter method
   public static double getTX() {
       return tx;
   }

   // ty için getter method
   public static double getTY() {
       return ty;
   }

   public void autoPositionToAllTags() {
       int[] priorityTags = {1, 2, 3, 4, 5, 6, 7, 8, 9};
       for (int tagId : priorityTags) {
           if (canSee) {
               // Hedef takip mantığı
               if (Math.abs(tx) > 1.0) {  // 1 derece tolerans
                   if (tx > 0) {
                       // Sağa dön
                   } else {
                       // Sola dön
                   }
               }
               
               if (Math.abs(ty) > 1.0) {  // 1 derece tolerans
                   // Dikey hizalama hesaplamaları
               }
           }
       }
   }

   @Override
   public void periodic() {
       // Değerleri periyodik olarak güncelle
       tx = LimelightHelpers.getTX(RobotConstants.LLName);
       ty = LimelightHelpers.getTY(RobotConstants.LLName);
       targetArea = LimelightHelpers.getTA(RobotConstants.LLName);

       if(targetArea == 0){
           canSee = false;
       }
       else{
           canSee = true;
       }

       // SmartDashboard'a değerleri gönder
       SmartDashboard.putNumber("LimelightX", tx);
       SmartDashboard.putNumber("LimelightY", ty);
       SmartDashboard.putNumber("LimelightArea", targetArea);
       SmartDashboard.putNumber("GetDistance", getDistance());
       SmartDashboard.putBoolean("cansee?", canSee);
   }

   @Override
   public void simulationPeriodic() {
   }

   public static int roundDistance(){
       //takes distance (currently subed for .getTa), and rounds it to the nearest whole number (casted to an int because round returns long)
       int distance = (int) round(getDistance());
       return distance;
   }

   public static double getDistance(){
       //https://docs.wpilib.org/en/latest/docs/software/vision-processing/introduction/identifying-and-processing-the-targets.html#distance
       //uses this equation ^
       //distance = (targetheight - cameraheight) / tan(cameraangle + Ty)
       double distance = (ShooterConstans.ApTagHeight - ShooterConstans.CamHeight) / 
                        Math.tan((ShooterConstans.CamAngle + (ty)) * (Math.PI/180));
       return distance;
   }

   // Hedef takibi için yardımcı metod
   public void trackTarget() {
       if (canSee) {
           if (Math.abs(tx) > 1.0) {  // 1 derece tolerans
               if (tx > 0) {
                   // Sağa dön
                   System.out.println("Sağa dön: " + tx + " derece");
               } else {
                   // Sola dön
                   System.out.println("Sola dön: " + tx + " derece");
               }
           }
           
           if (Math.abs(ty) > 1.0) {
               System.out.println("Dikey offset: " + ty + " derece");
           }
       }
   }
}
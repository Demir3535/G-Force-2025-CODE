package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstans;
import frc.robot.RobotConstants;
import frc.robot.LimelightHelpers;
import static java.lang.Math.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LimelightSubsystem extends SubsystemBase {

    public static boolean canSee;
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");



    @Override
    public void periodic() {
        //read values periodically
        double x = LimelightHelpers.getTX(RobotConstants.LLName);
        double y = LimelightHelpers.getTY(RobotConstants.LLName);
        double area = LimelightHelpers.getTA(RobotConstants.LLName);

        if(area == 0){
            canSee = false;
        }
        else{
            canSee = true;
        }
        //boolean tv = RobotContainer.getTv();

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("GetDistance", getDistance());
        SmartDashboard.putBoolean("cansee?", canSee);

        // Writing new AprilTag ids
       // SmartDashboard.putNumber("AprilTag ID", getTagID());
    
       // var poseEstimate = getPoseEstimate();
        //if (poseEstimate != null) {
          //  SmartDashboard.putNumber("Tag Count", poseEstimate.tagCount);
           // SmartDashboard.putNumber("Avg Tag Distance", poseEstimate.avgTagDist);
           // SmartDashboard.putNumber("Pose Latency", poseEstimate.latency);
            
            // Robot position
            //Pose2d pose = poseEstimate.pose;
            //SmartDashboard.putNumber("Robot X", pose.getX());
            //  SmartDashboard.putNumber("Robot Y", pose.getY());
            //SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());
            // }

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
        var y = LimelightHelpers.getTY(RobotConstants.LLName);
        double distance = (ShooterConstans.ApTagHeight - ShooterConstans.CamHeight) / Math.tan((ShooterConstans.CamAngle + (y)) * (Math.PI/180));
        return distance;
   }
}
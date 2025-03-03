package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    // NetworkTable for the Limelight camera
    NetworkTable table;
    
    // NetworkTable entries for target validity, horizontal angle, vertical angle, target area, and AprilTag ID
    NetworkTableEntry tv; // Target validity (0: no target, 1: target detected)
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid; // AprilTag ID
    
    // Variables for storing the Limelight NetworkTable stream updates
    double xDisplacement;
    double yDisplacement;
    double targetArea;
    int targetID; // To store AprilTag ID
    double x;
    double y;
    double area;
    double desiredDistance;
    double DESIRED_TARGET;
    
    /** Creates a new LimelightSubsystem. */
    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable(RobotConstants.LLName);
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid"); // Initialize AprilTag ID entry
        DESIRED_TARGET = LimelightConstants.DESIRED_TARGET;
        
        // Set the pipeline to use (default to pipeline 0)
        setPipeline(0);
    }
    
    /**
     * Set the active pipeline on the Limelight
     * @param pipeline Pipeline number (0-9)
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Get the current active pipeline
     * @return Current pipeline number
     */
    public int getPipeline() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }
    
    /**
     * Get the horizontal offset from crosshair to target in degrees
     * @return Horizontal offset in degrees
     */
    public double getTx() {
        return tx.getDouble(0.0);
    }
    
    /**
     * Get the vertical offset from crosshair to target in degrees
     * @return Vertical offset in degrees
     */
    public double getTy() {
        return ty.getDouble(0.0);
    }
    
    /**
     * Get the target area (0-100% of image)
     * @return Target area percentage
     */
    public double getTa() {
        return ta.getDouble(0.0);
    }
    
    /**
     * Get the AprilTag ID
     * @return AprilTag ID, or 0 if none detected
     */
    public int getTargetID() {
        return (int) tid.getDouble(0.0);
    }
    
    /**
     * Calculate the steering adjustment based on tx value
     * @return Steering adjustment value
     */
    public double getSteer() {
        double tx = getTx();
        return -tx * LimelightConstants.STEER_K; // Negatif işaret ekledik
    }
    
    /**
     * Check if the desired target area has been reached
     * @return True if target area is at or above the desired value
     */
    public boolean targetAreaReached() {
        return getTa() >= DESIRED_TARGET;
    }
    
    /**
     * Get the horizontal displacement from crosshair to target
     * @return Horizontal displacement in degrees
     */
    public double getXDisplacement() {
        xDisplacement = getTx();
        return xDisplacement;
    }
    
    /**
     * Check if the Limelight has any valid targets
     * @return True if Limelight has a valid target
     */
    public boolean hasTargets() {
        return tv.getDouble(0.0) == 1.0; // tv value of 1 means target detected
    }
    
    /**
     * Get the vertical displacement from crosshair to target
     * @return Vertical displacement in degrees
     */
    public double getYDisplacement() {
        yDisplacement = getTy();
        return yDisplacement;
    }
    
    /**
     * Get the target area
     * @return Target area as percentage of image
     */
    public double getTargetArea() {
        targetArea = getTa();
        return targetArea;
    }
    
    /**
     * Calculate the distance to target using trigonometry
     * This requires proper calibration of camera height and angle
     * @return Distance to target in meters
     */
    public double getDistanceToTarget() {
        // Ty değerini kullanarak trigonometri ile mesafeyi hesapla
        double targetOffsetAngle_Vertical = getTy();
        
        // Mesafe hesaplama formülü
        double angleToGoalRadians = LimelightConstants.CAMERA_PITCH_RADIANS + Math.toRadians(targetOffsetAngle_Vertical);
        double distanceFromLimelightToGoalMeters = (LimelightConstants.TARGET_HEIGHT_METERS - LimelightConstants.CAMERA_HEIGHT_METERS) / 
                                                  Math.tan(angleToGoalRadians);
        
        return distanceFromLimelightToGoalMeters;
    }
    
    /**
     * Toggle the Limelight's LEDs
     * @param enabled True to turn on LEDs, false to turn off
     */
    public void setLedMode(boolean enabled) {
        int mode = enabled ? 3 : 1; // 3 = force on, 1 = force off
        table.getEntry("ledMode").setNumber(mode);
    }
    
    @Override
    public void periodic() {
        // Read values periodically
        x = getTx();
        y = getTy();
        area = getTa();
        targetID = getTargetID();
        
        // Post to SmartDashboard periodically
        SmartDashboard.putBoolean("Limelight Has Target", hasTargets());
        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);
        SmartDashboard.putNumber("Limelight Area", area);
        SmartDashboard.putNumber("Limelight Target ID", targetID);
        
        // Additional diagnostic information
        SmartDashboard.putNumber("Limelight Pipeline", getPipeline());
        if (hasTargets()) {
            SmartDashboard.putNumber("Target Distance (m)", getDistanceToTarget());
        }
    }
}
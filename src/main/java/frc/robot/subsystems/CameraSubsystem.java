package frc.robot.subsystems;
import frc.robot.RobotConstants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;

public class CameraSubsystem extends SubsystemBase {
    // NetworkTable for the Limelight camera
    private final NetworkTable table;
    // NetworkTable entries for horizontal angle, vertical angle, and target area
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    // Variables for storing the Limelight NetworkTable stream updates
    private double x;
    private double y;
    private double area;

    private final PIDController drivePidController;
    private final PIDController turnPidController;

    /** Creates a new CameraSubsystem. */
    public CameraSubsystem() {
        // Initialize NetworkTable for Limelight
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        drivePidController = new PIDController(LimelightConstants.LINEAR_P, 0, LimelightConstants.LINEAR_D);
        turnPidController = new PIDController(LimelightConstants.ANGULAR_P, 0, LimelightConstants.ANGULAR_D);
    }

    public boolean hasTargets() {
        double area = ta.getDouble(0.0);
        return area > 0; // Hedef alanı sıfırdan büyükse hedef var demektir
    }

    public double getDriveSpeed() {
        double driveSpeed = 0;
        if (hasTargets()) {
            // Calculate range using Limelight's vertical angle (ty)
            double range = (LimelightConstants.TARGET_HEIGHT_METERS - LimelightConstants.CAMERA_HEIGHT_METERS) /
                Math.tan(LimelightConstants.CAMERA_PITCH_RADIANS + Units.degreesToRadians(ty.getDouble(0.0)));
            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort *increases* range
            driveSpeed = -drivePidController.calculate(range, LimelightConstants.GOAL_RANGE_METERS);
        }
        return driveSpeed;
    }

    public double getTurnSpeed() {
        double turnSpeed = 0;
        if (hasTargets()) {
            // Use Limelight's horizontal angle (tx) for turning
            turnSpeed = -turnPidController.calculate(tx.getDouble(0.0), 0);
        }
        return turnSpeed;
    }

    @Override
    public void periodic() {
        // Read values periodically
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        // Post to SmartDashboard periodically
        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);
        SmartDashboard.putNumber("Limelight Area", area);
    }
}
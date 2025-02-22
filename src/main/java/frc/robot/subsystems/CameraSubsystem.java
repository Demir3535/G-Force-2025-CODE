package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
public class CameraSubsystem extends SubsystemBase {
    // NetworkTable for the Limelight camera
    NetworkTable table;
    // NetworkTable entries for horizontal angle, vertical angle, and target area
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    // Variables for storing the Limelight NetworkTable stream updates
    double x;
    double y;
    double area;

    // Constants
    final double CAMERA_HEIGHT_METERS;
    final double TARGET_HEIGHT_METERS;
    final double CAMERA_PITCH_RADIANS;
    final double GOAL_RANGE_METERS;
    final double LINEAR_P;
    final double LINEAR_D;
    PIDController drivePidController;
    final double ANGULAR_P;
    final double ANGULAR_D;
    PIDController turnPidController;

    /** Creates a new CameraSubsystem. */
    public CameraSubsystem() {
        // Initialize NetworkTable for Limelight
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        // Constants such as camera and target height stored. Change per robot and goal!
        CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
        TARGET_HEIGHT_METERS = Units.feetToMeters(0);
        // Angle between horizontal and the camera.
        CAMERA_PITCH_RADIANS = Units.degreesToRadians(45);

        // How far from the target we want to be
        GOAL_RANGE_METERS = Units.feetToMeters(3);

        // PID constants should be tuned per robot
        LINEAR_P = 0.1;
        LINEAR_D = 0.0;
        drivePidController = new PIDController(LINEAR_P, 0, LINEAR_D);

        ANGULAR_P = 0.1;
        ANGULAR_D = 0.0;
        turnPidController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    }

    public boolean hasTargets() {
        double area = ta.getDouble(0.0);
        return area > 0; // Hedef alanı sıfırdan büyükse hedef var demektir
    }

    public double getDriveSpeed() {
        double driveSpeed = 0;

        if (hasTargets()) {
            // Calculate range using Limelight's vertical angle (ty)
            double range =
                    (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) /
                    Math.tan(CAMERA_PITCH_RADIANS + Units.degreesToRadians(ty.getDouble(0.0)));

            // Use this range as the measurement we give to the PID controller.
            // -1.0 required to ensure positive PID controller effort _increases_ range
            driveSpeed = -drivePidController.calculate(range, GOAL_RANGE_METERS);
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
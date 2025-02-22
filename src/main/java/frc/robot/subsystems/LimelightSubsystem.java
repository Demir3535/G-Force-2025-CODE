package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
    // NetworkTable for the Limelight camera
    NetworkTable table;
    // NetworkTable entries for horizontal angle, vertical angle, target area, and AprilTag ID
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid; // AprilTag ID'si

    // Variables for storing the Limelight NetworkTable stream updates
    double xDisplacement;
    double yDisplacement;
    double targetArea;
    int targetID; // AprilTag ID'sini saklamak için

    double x;
    double y;
    double area;

    double desiredDistance;
    double DESIRED_TARGET;

    /** Creates a new LimelightSubsystem. */
    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tid = table.getEntry("tid"); // AprilTag ID'sini başlat

        DESIRED_TARGET = 4;
    }

    public double getTx() {
        return tx.getDouble(0.0);
    }

    public double getTy() {
        return ty.getDouble(0.0);
    }

    public double getTa() {
        return ta.getDouble(0.0);
    }

    // AprilTag ID'sini döndürür
    public int getTargetID() {
        return (int) tid.getDouble(0.0);
    }

    public boolean targetAreaReached() {
        double area = ta.getDouble(0.0);
        return area >= DESIRED_TARGET;
    }

    public double getSteer() {
        double tx = getTx();
        final double STEER_K = 0.06; // Dönüş hassasiyeti
        return tx * STEER_K;
    }

    public double getXDisplacement() {
        xDisplacement = tx.getDouble(0.0);
        return xDisplacement;
    }
    public boolean hasTargets() {
      double area = ta.getDouble(0.0);
      return area > 0; // Hedef alanı sıfırdan büyükse hedef var demektir
  }
    public double getYDisplacement() {
        yDisplacement = ty.getDouble(0.0);
        return yDisplacement;
    }

    public double getTargetArea() {
        targetArea = ta.getDouble(0.0);
        return targetArea;
    }

    @Override
    public void periodic() {
        // Read values periodically
        x = getTx();
        y = getTy();
        area = getTa();
        targetID = getTargetID();

        // Post to SmartDashboard periodically
        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);
        SmartDashboard.putNumber("Limelight Area", area);
        SmartDashboard.putNumber("Limelight Target ID", targetID); // AprilTag ID'sini yaz
    }
}
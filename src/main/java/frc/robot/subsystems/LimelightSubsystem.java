// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {

  // NetworkTable for the Limelight camera (contains list of NetworkTableEntries, which are bits of a continuous data stream)
  NetworkTable table;
  // NetworkTable data stream (aka an entry) for horizontal angle displacement, vertical angle displacement, and target area
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  // Variables for storing the Limelight NetworkTable stream updates
  double xDisplacement;
  double yDisplacement;
  double targetArea;

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

    // desiredDistance = 3;

    DESIRED_TARGET = 4;


    // //read values periodically
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    // //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

    
  }

  public double getTx() {
    double x = tx.getDouble(0.0);
    return x;
  }

  // public double getTa() {
  //   double a = ta.getDouble(0.0);
  //   return a;
  // }

  public boolean targetAreaReached() {
    double area = ta.getDouble(0.0);
    if (area >= DESIRED_TARGET) {
      return true;
    }
    return false;
  }

  public double getSteer() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    final double STEER_K = 0.06;  // robot sallaniyorsa azaltin 
    // Start with proportional steering
    double steer = tx * STEER_K;
    return steer;
  }



  // public double getDrive() {
  //   double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  //   final double DRIVE_K = 0.26;

  //   // try to drive forward until the target area reaches our desired area
  //   double drive = (DESIRED_TARGET - ta) * DRIVE_K;

  //   return drive;
  // }

  

  

  // public double getDistance() {
  //   double targetOffsetAngle_Vertical = ty.getDouble(0.0);

  //   // how many degrees back is your limelight rotated from perfectly vertical?
  //   double llAngleDegrees = 0.0; 

  //   // distance from the center of the Limelight lens to the floor
  //   double llHeightInches = 48.0; 

  //   // distance from the target to the floor
  //   double goalHeightInches = 46.0; 

  //   double angleToGoalDegrees = llAngleDegrees + targetOffsetAngle_Vertical;
  //   double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  //   //calculate distance
  //   double distanceFromLimelightToGoalInches = (goalHeightInches - llHeightInches) / Math.tan(angleToGoalRadians);
  
  //   return distanceFromLimelightToGoalInches;
  // }



  // public double getDriveAdjust() {
  //   double KpDistance = -0.1f;  // Proportional control constant for distance
  //   double currentDistance = getDistance();  

  //   double distanceError = 3 - currentDistance;
  //   double drivingAdjust = KpDistance * distanceError;
    
  //   if (checkDistanceError()) {
  //     return drivingAdjust;
  //   }
  //   return 0;
  // }

  // public boolean checkDistanceError() {
  //   double currentDistance = getDistance();  

  //   double distanceError = 3 - currentDistance;
  //   if (distanceError <= 0.5) {
  //     return false;
  //   }
  //   return true;
  // }

  // This is a double getter method for retrieving the horizontal angle displacement
  public double getXDisplacement() {
    xDisplacement = tx.getDouble(0.0);
    return xDisplacement;
  }

  // This is a double getter method for retrieving the vertical angle displacement
  public double getYDisplacement() {
    yDisplacement = ty.getDouble(0.0);
    return yDisplacement;
  }

  // This is a double getter method for retrieving the target area
  public double getTargetArea() {
    targetArea = ta.getDouble(0.0);
    return targetArea;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    SmartDashboard.putNumber("area", area);

  }

}
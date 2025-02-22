// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class CameraSubsystem extends SubsystemBase {

  PhotonCamera camera;
  boolean hasTargets;
  List<PhotonTrackedTarget> targets;
  PhotonTrackedTarget target;

  double yaw;
  double pitch;
  double area;
  double skew;
  Transform3d notePose;
  List<TargetCorner> corners;

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

    camera = new PhotonCamera("Live_Streamer_CAM_313");

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
    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();
    return hasTargets;
  }

  public double getDriveSpeed() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    double driveSpeed;
    // double turnSpeed;

    if (result.hasTargets()) {
        // First calculate range
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result.getBestTarget().getPitch()));

        // Use this range as the measurement we give to the PID controller.
        // -1.0 required to ensure positive PID controller effort _increases_ range
        driveSpeed = -drivePidController.calculate(range, GOAL_RANGE_METERS);

        // // Also calculate angular power
        // // -1.0 required to ensure positive PID controller effort _increases_ yaw
        // turnSpeed = -turnPidController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
        // If we have no targets, stay still.
        driveSpeed = 0;
        // turnSpeed = 0;
    }

    return driveSpeed;
  }

  public double getTurnSpeed() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    // double driveSpeed;
    double turnSpeed;

    if (result.hasTargets()) {
        // First calculate range
        double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result.getBestTarget().getPitch()));

        // // Use this range as the measurement we give to the PID controller.
        // // -1.0 required to ensure positive PID controller effort _increases_ range
        // driveSpeed = -drivePidController.calculate(range, GOAL_RANGE_METERS);

        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        turnSpeed = -turnPidController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
        // If we have no targets, stay still.
        // driveSpeed = 0;
        turnSpeed = 0;
    }

    return turnSpeed;
  }


  @Override
  public void periodic() {
    // // Query the latest result from PhotonVision
    // var result = camera.getLatestResult();

    // // Check if the latest result has any targets.
    // hasTargets = result.hasTargets();

    // // Get a list of currently tracked targets.
    // targets = result.getTargets();

    // // Get the current best target.
    // target = result.getBestTarget();

    // // Get information from target (game piece).
    // yaw = target.getYaw();
    // pitch = target.getPitch();
    // area = target.getArea();
    // skew = target.getSkew();
    // notePose = target.getBestCameraToTarget();
    // corners = target.getDetectedCorners();

  }
}
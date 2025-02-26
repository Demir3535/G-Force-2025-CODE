package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class RobotConstants {


        public static final String LLName = "limelight";

        public static final class ScoringConstants { // Class containing constant values for scoring positions
                public static final class BlueAlliance { // Specific positions for the Blue alliance
                        public static final List<Pose2d> REEF_SIDE_POSES = List.of( // List of Reef (field edge) positions
                                        new Pose2d(2.961, 4, new Rotation2d(Math.toRadians(0))), // 1st Position: X:2.961m, Y:4m, Angle:0°
                                        new Pose2d(3.712, 2.723, new Rotation2d(Math.toRadians(30))), // 2nd Position: X:3.712m, Y:2.723m, Angle:30°
                                        new Pose2d(5.253, 2.656, new Rotation2d(Math.toRadians(60))), // 3rd Position: X:5.253m, Y:2.656m, Angle:60°
                                        new Pose2d(6, 4, new Rotation2d(Math.toRadians(90))), // 4th Position: X:6m, Y:4m, Angle:90°
                                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(120))), // 5th Position: X:0m, Y:0m, Angle:120°
                                        new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))); // 6th Position: X:0m, Y:0m, Angle:0°

                        public static final List<Pose2d> HP_POSES = List.of( // Human Player station positions
                                        new Pose2d(1.2, 7, new Rotation2d(Units.degreesToRadians(125))), // Upper HP position: X:1.2m, Y:7m, Angle:125°
                                        new Pose2d(1.2, 1, new Rotation2d(Units.degreesToRadians(-125)))); // Lower HP position: X:1.2m, Y:1m, Angle:-125°
                }
        }
        public static final class MotorSpeeds{
                public static final double eP = 0.075;
                public static final double eI = 0;
                public static final double eD = 0;
                //slower pid values for going down
                public static final double edP = 0.01;
                public static final double edI = 0;
                public static final double edD = 0;
                //encoder values for each setpoint, assuming bottom is 0
                //top of elevator is around encoder count = 28
                //index to close to zero for faster drop (motors will fall the rest of the way to prevent damage)
                public static final double elevatorL0 = 1;
                public static final double elevatorL1 = -20;
                public static final double elevatorL2 = -40;
                public static final double elevatorL3 = -60;
                public static final double elevatorL4 = -15;
            
                public static final double elevatorSpeed = .2;
                public static final double elevatorSpeedDown = .085;
            
                public static final double coralSpeed = .2;
            
              }

        public static final class ElevatorConstants {   //TODO elevator settings
                public static final double ELEVATOR_MAX_HEIGHT = -112;
                public static final double ELEVATOR_MIN_HEIGHT = -0.2;

                public static final class HeightSetpoints {
                      
                        public static final double HOME = -3;
                        public static final double HP = -50;
                        public static final class Coral {
                                public static final double L1 = -20;
                                public static final double L2 = -46;
                                public static final double L3 = -60;
                                public static final double HOME = 0;
                        }
                        public static final class Algae {
                                public static final double L2 = -42;
                                public static final double L3 = -70;
                        }
                       
        
                }
               

                public static final double MAX_MOTOR_RPM = 210.2;
                public static final double MAX_MOTOR_ACCELERATION = 3000.0;

                public static final class SimConstants {
                        public static final double L1 = 0.2;
                        public static final double L2 = 0.3;
                        public static final double L3 = 0.55;
                        public static final double HP = -50;

                }


                public static final double P = 2.5;
                public static final double I = 0;
                public static final double D = 0;

        }
       
        public static final class LimelightConstants {   // TODO changes these if theres a error for limelight
                public static final double STEER_K = 0.06;
                public static final double DESIRED_TARGET = 4.0;
                public static final double MAX_DRIVE_SPEED = 0.7;
                public static final double MAX_TURN_SPEED = 0.7;
                public static final double ACCEPTABLE_TX_ERROR = 1.0;
                public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
                public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(0);
                public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(45);
                public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);
                public static final double LINEAR_P = 0.1;
                public static final double LINEAR_D = 0.0;
                public static final double ANGULAR_P = 0.1;
                public static final double ANGULAR_D = 0.0;
          
         }


        public static final class ShooterConstants {
                public static final double MAX_MOTOR_RPM = 500.0;
                public static final double MAX_MOTOR_ACCELERATION = 500.0;
                public static final double SHOOTER_SPEED = 0.5; // Adjust the desired speed

                public static final int LIMIT_PORT = 1;

        }

        public static final class ClimbConstans {
                public static final double MAX_MOTOR_RPM = 500.0;
                public static final double MAX_MOTOR_ACCELERATION = 500.0;

                public static final double CLIMB_SPEED = 0.5; // Adjust the desired speed

        }


        public static final class WristConstants {    //TODO wrist constants for robot
                public static final double WRIST_MIN_ANGLE = 0.0;
                public static final double ELEVATOR_MAX_ANGLE = 30.0;

                public static final class AngleSetpoints {
                        public static final double HOME = -5;
                        public static final double HP = -10.0;

                        public static final class Coral {
                                public static final double L1 = -10;
                                public static final double L2 = -3;
                                public static final double L3 = -5;
                        }

                        public static final class Algae {
                                public static final double L2 = 12;
                                public static final double L3 = 13.5;
                        }
                }

                public static final double MAX_MOTOR_RPM = 3000.0;
                public static final double MAX_MOTOR_ACCELERATION = 1000.0;
      }

        public static final class DrivetrainConstants {
              
                public static final double tP = 0.1;
                public static final double tI = 0;
                public static final double tD = 0.02;

              
                public static final double FRONT_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double FRONT_RIGHT_VIRTUAL_OFFSET_RADIANS = 0; // -We do not apply an offset to the
                                                                                   // CANcoder
                                                                                   // angle, we just zero the encoders
                                                                                   // with the
                                                                                   // wheels forward
                                                                                   // -In radians
                public static final double REAR_LEFT_VIRTUAL_OFFSET_RADIANS = 0;
                public static final double REAR_RIGHT_VIRTUAL_OFFSET_RADIANS = 0;

                // Driving Parameters - Note that these are not the maximum capable speeds of
                // the robot, rather the allowed maximum speeds
                public static final double MAX_SPEED_METERS_PER_SECOND = 6.0; // 4.42; //4.8;
                public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI; // radians per second

                public static final double DIRECTION_SLEW_RATE = 25;// radians per second
                public static final double MAGNITUDE_SLEW_RATE = 25;// percent per second (1 = 100%)
                public static final double ROTATIONAL_SLEW_RATE = 10;// percent per second (1 = 100%)

                // Chassis configuration

                public static final double DRIVE_BASE_RADIUS_METERS = 0.42; // measurement from center point of robot
                                                                            // to the
                                                                            // center of one of the wheels. (use the
                                                                            // CAD)

                public static final double LEFT_RIGHT_DISTANCE_METERS = Units.inchesToMeters(27); // Distance between
                                                                                                  // centers of
                                                                                                  // right
                // and left wheels on robot

                public static final double FRONT_BACK_DISTANCE_METERS = Units.inchesToMeters(27);// Distance between
                                                                                                 // front and
                                                                                                 // back
                // wheels on robot

                public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                                new Translation2d(LEFT_RIGHT_DISTANCE_METERS / 2, FRONT_BACK_DISTANCE_METERS / 2),
                                new Translation2d(LEFT_RIGHT_DISTANCE_METERS / 2, -FRONT_BACK_DISTANCE_METERS / 2),
                                new Translation2d(-LEFT_RIGHT_DISTANCE_METERS / 2, FRONT_BACK_DISTANCE_METERS / 2),
                                new Translation2d(-LEFT_RIGHT_DISTANCE_METERS / 2, -FRONT_BACK_DISTANCE_METERS / 2));

                public static final int GYRO_ORIENTATION = -1; // 1 for upside down, -1 for right side up.

                public static final boolean FIELD_RELATIVE = true;
        }

        public static class RobotState {
                private static boolean canRotate = false;

                public static Command setCanRotate(boolean rotate) {
                        return new InstantCommand(() -> canRotate = rotate);
                }

                public static boolean getCanRotate() {
                        return canRotate;
                }
        }

        public static final class SwerveModuleConstants {

                public static final double TRANSLATION_P = 1.0;
                public static final double ROT_MOTION_P = 0.0;

                public static final double TRANSLATION_I = 0.0;
                public static final double ROT_MOTION_I = 0.0;

                public static final double TRANSLATION_D = 0.0;
                public static final double ROT_MOTION_D = 0.0;

                public static final double FREE_SPEED_RPM = 5676;

                public static final int DRIVING_MOTOR_PINION_TEETH = 14;

                public static final boolean TURNING_ENCODER_INVERTED = true;

                public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
                public static final double WHEEL_DIAMETER_METERS = 0.1016;
                public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
                public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 17 * 50)
                                / (DRIVING_MOTOR_PINION_TEETH * 15 * 27);
                public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
                                * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;

                public static final double DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION = (WHEEL_DIAMETER_METERS
                                * Math.PI) / DRIVING_MOTOR_REDUCTION; // meters, per rotation
                public static final double DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM = ((WHEEL_DIAMETER_METERS
                                * Math.PI) / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second, per RPM

                public static final double TURNING_MOTOR_REDUCTION = 150.0 / 7.0; // Ratio between internal relative
                                                                                  // encoder and
                                                                                  // the absolute encoder

                public static final double TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION = (2 * Math.PI)
                                / TURNING_MOTOR_REDUCTION; // radians, per rotation
                public static final double TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM = (2 * Math.PI)
                                / TURNING_MOTOR_REDUCTION / 60.0; // radians per second, per RPM

                public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT_RADIANS = 0; // radians
                public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT_RADIANS = (2 * Math.PI); // radians

                // These PID constants relate to the movement and acceleration of the swerve
                // motors themselfs.
                public static final double DRIVING_P = 0.07;
                public static final double DRIVING_I = 0;
                public static final double DRIVING_D = 0;
                public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;
                public static final double DRIVING_MIN_OUTPUT_NORMALIZED = -1;
                public static final double DRIVING_MAX_OUTPUT_NORMALIZED = 1;

                public static final double TURNING_P = 1.25;
                public static final double TURNING_I = 0;
                public static final double TURNING_D = 0;
                public static final double TURNING_FF = 0;
                public static final double TURNING_MIN_OUTPUT_NORMALIZED = -1;
                public static final double TURNING_MAX_OUTPUT_NORMALIZED = 1;

                public static final IdleMode DRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
                public static final IdleMode TURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

                public static final int DRIVING_MOTOR_CURRENT_LIMIT_AMPS = 40; // amps
                public static final int TURNING_MOTOR_CURRENT_LIMIT_AMPS = 20; // amps

        }

        public static interface PortConstants {

                public static class CAN {
                        public static final int FRONT_LEFT_DRIVING = 5;
                        public static final int REAR_LEFT_DRIVING = 7;
                        public static final int FRONT_RIGHT_DRIVING = 6;
                        public static final int REAR_RIGHT_DRIVING = 8;

                        public static final int FRONT_LEFT_TURNING = 9;
                        public static final int REAR_LEFT_TURNING = 11;
                        public static final int FRONT_RIGHT_TURNING = 23;
                        public static final int REAR_RIGHT_TURNING = 45;

                        public static final int FRONT_LEFT_CANCODERID = 1;
                        public static final int FRONT_RIGHT_CANCODERID = 2;
                        public static final int REAR_LEFT_CANCODERID = 3;
                        public static final int REAR_RIGHT_CANCODERID = 4;

                        public static final int ELEVATOR_MOTOR_1 = 12;
                        public static final int ELEVATOR_MOTOR_2 = 10;
                       

                        public static final int SHOOTER_MOTOR_1 = 22;

                        
                        public static final int WRIST_MOTOR = 18;

                        public static final int CLIMB_MOTOR = 19;

                        public static final int CLAW_MOTOR = 21;
                        public static final int PDH = 20;
                        public static double limelightAdjust;

                }

                
                public static class DIO {
                        
                        public static final int SHOOTER_DISTANCE_SENSOR = 0; // exp port number 
                }

                public static final class PWM {
                        public static final int LED_CONTROLLER = 0; // led port for roborio
                }

                public static class Controller {
                        public static final double JOYSTICK_AXIS_THRESHOLD = 0.2;
                        public static final int DRIVE_JOYSTICK = 1;
                        public static final int OPERATOR_JOYSTICK = 0;

                        // Joystick Axis
                        public static final int DRIVE_COMMAND_X_AXIS = 0;
                        public static final int DRIVE_COMMAND_Y_AXIS = 1;
                        public static final int DRIVE_COMMAND_ROT_AXIS = 2; // 2 for the flight controller, 4 for
                                                                            // xbox/gamepad

                        // Manual control axis for operator
                        public static final int ELEVATOR_MANUAL_CONTROL = 1;
                        public static final int WRIST_MANUAL_CONTROL = 3;
                        public static final double kDriveDeadband = 0.3;
                }
        }

        public static final class AutonomousConstants {
                public static final boolean FLIP_PATHPLANNER_AUTOS = false;

                public static final double FIELD_LENGTH_INCHES = 54 * 12 + 1; // 54ft 1in
                public static final double FIELD_WIDTH_INCHES = 26 * 12 + 7; // 26ft 7in

        }

        public static final class TeleopConstants {  // TODO speed settings 
                public static final double MAX_SPEED_PERCENT = 1; // ex: 0.4 -> 40%
        }

        public static final class PathPlannerConstants {  // TODO pathplanner speeds for autonomous 
                public static final Alliance DEFAULT_ALLIANCE = Alliance.Blue;

                public static final double kMaxAngularAcceleration = 4 * Math.PI;
                public static final double kMaxAccelerationMetersPerSecondSquared = 3.00;

                public static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
                                DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND,
                                PathPlannerConstants.kMaxAccelerationMetersPerSecondSquared,
                                DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                5 * Math.PI);

                public static final double MAX_VELOCITY = 1.0; // Meters per second
                public static final double MAX_ACCELERATION = 1.0; // Meters per second squared
                public static final double MAX_ANGULAR_SPEED = 180.0; // Degrees per second
                public static final double MAX_ANGULAR_ACCELERATION = 270.0; // Degrees per second squared
        }

        public static final class SubsystemEnabledConstants {
                public static final boolean DRIVE_SUBSYSTEM_ENABLED = true;
                public static final boolean VISION_SUBSYSTEM_ENABLED = false;
        }
}
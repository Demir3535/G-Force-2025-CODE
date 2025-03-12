package frc.robot.subsystems.drive;

import java.util.Optional;
import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotConstants;
import frc.robot.RobotState;
import frc.robot.utils.GforceUtils;
import frc.robot.RobotConstants.DrivetrainConstants;
import frc.robot.RobotConstants.SubsystemEnabledConstants;
import frc.robot.RobotContainer.UserPolicy;
import frc.robot.subsystems.drive.swerve.SwerveModule;
import frc.robot.subsystems.drive.swerve.SwerveModuleSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import frc.robot.utils.SwerveUtils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * The {@code Drivetrain} class contains fields and methods pertaining to the
 * function of the drivetrain.
 * 
 */

public class DriveSubsystem extends SubsystemBase {
    private SwerveModuleSim[] swerveModuleSims = new SwerveModuleSim[4];
    private SwerveModule[] swerveModules = new SwerveModule[4];
    RobotConfig config;
    private static AHRS m_gyro;

    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
    PIDController turningPID = new PIDController(DrivetrainConstants.tP, DrivetrainConstants.tI,
            DrivetrainConstants.tD);

    private final PIDController xController = new PIDController(1.0, 0, 0);
    private final PIDController yController = new PIDController(1.0, 0, 0);
    private final PIDController rotationController = new PIDController(1.0, 0, 0);

    // YENI EKLENDI: Pozisyonlanma
    private static final double POSITION_TOLERANCE = 0.05; // 5 cm
    private static final double ROTATION_TOLERANCE = 2.0; // 2 derece

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DrivetrainConstants.MAGNITUDE_SLEW_RATE);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.ROTATIONAL_SLEW_RATE);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    private Rotation2d m_trackedRotation = new Rotation2d();

    private static SwerveDrivePoseEstimator m_odometry;

    private double fakeGyro = 0;
    Field2d field = new Field2d();

    // temp:
    StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

    /** Creates a new Drivetrain. */
    public DriveSubsystem() {

        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {

            if (RobotBase.isSimulation()) {
                // Make simulated swerve modules
                swerveModuleSims[0] = new SwerveModuleSim(); // Front Left
                swerveModuleSims[1] = new SwerveModuleSim(); // Front Right
                swerveModuleSims[2] = new SwerveModuleSim(); // Rear Left
                swerveModuleSims[3] = new SwerveModuleSim(); // Rear Right

                m_odometry = new SwerveDrivePoseEstimator(
                        DrivetrainConstants.DRIVE_KINEMATICS,
                        Rotation2d.fromDegrees(fakeGyro),
                        new SwerveModulePosition[] {
                                swerveModuleSims[0].getPosition(),
                                swerveModuleSims[1].getPosition(),
                                swerveModuleSims[2].getPosition(),
                                swerveModuleSims[3].getPosition()
                        }, new Pose2d());

            } else {
                // If the code is actually running on the robot, make real swerve module
                // instances.
                swerveModules[0] = new SwerveModule(// Front Left
                        RobotConstants.PortConstants.CAN.FRONT_LEFT_DRIVING,
                        RobotConstants.PortConstants.CAN.FRONT_LEFT_TURNING,
                        RobotConstants.PortConstants.CAN.FRONT_LEFT_CANCODERID, true); //swerve error çıkarsa burası TODO

                swerveModules[1] = new SwerveModule( // Front Right
                        RobotConstants.PortConstants.CAN.FRONT_RIGHT_DRIVING,
                        RobotConstants.PortConstants.CAN.FRONT_RIGHT_TURNING,
                        RobotConstants.PortConstants.CAN.FRONT_RIGHT_CANCODERID, false);

                swerveModules[2] = new SwerveModule( // Rear Left
                        RobotConstants.PortConstants.CAN.REAR_LEFT_DRIVING,
                        RobotConstants.PortConstants.CAN.REAR_LEFT_TURNING,
                        RobotConstants.PortConstants.CAN.REAR_LEFT_CANCODERID, false);

                swerveModules[3] = new SwerveModule( // Rear Right
                        RobotConstants.PortConstants.CAN.REAR_RIGHT_DRIVING,
                        RobotConstants.PortConstants.CAN.REAR_RIGHT_TURNING,
                        RobotConstants.PortConstants.CAN.REAR_RIGHT_CANCODERID, false);

                m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

                m_gyro.reset();
                resetEncoders();

                m_odometry = new SwerveDrivePoseEstimator(
                        DrivetrainConstants.DRIVE_KINEMATICS,
                        Rotation2d.fromDegrees(DrivetrainConstants.GYRO_ORIENTATION * getGyroAngle()),
                        new SwerveModulePosition[] {
                                swerveModules[0].getPosition(),
                                swerveModules[1].getPosition(),
                                swerveModules[2].getPosition(),
                                swerveModules[3].getPosition()
                        }, new Pose2d());

            }
        }

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            rotationController.enableContinuousInput(-Math.PI, Math.PI);
            xController.setTolerance(POSITION_TOLERANCE);
            yController.setTolerance(POSITION_TOLERANCE);
            rotationController.setTolerance(ROTATION_TOLERANCE);

            turningPID.enableContinuousInput(-180, 180); // for degree values
            turningPID.setTolerance(1.0); // target tolerance
            // set PID value
            turningPID.setP(0.1);
            turningPID.setI(0.0);
            turningPID.setD(0.0);
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                DriveSubsystem::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> pathFollowDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                     // ChassisSpeeds. Also optionally outputs individual module
                                                     // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0), // Translation PID constants  //TODO PID IMPORTANT VALUES FOR PATHPLANNER
                        new PIDConstants(0.05, 0.0, 0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    private double getGyroAngle() {
        return RobotBase.isReal() ? m_gyro.getAngle() * DrivetrainConstants.GYRO_ORIENTATION : fakeGyro;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModuleSims.length];
        for (int i = 0; i < swerveModuleSims.length; i++) {
            states[i] = swerveModuleSims[i].getState();
        }
        return states;
    }

    private void putSmartDashboardData() {
        if (RobotBase.isReal()) {
            SmartDashboard.putData("Odometry Pose Field", field);
            SmartDashboard.putNumberArray("modules pose angles", new double[] {
                    swerveModules[0].getPosition().angle.getDegrees(),
                    swerveModules[1].getPosition().angle.getDegrees(),
                    swerveModules[2].getPosition().angle.getDegrees(),
                    swerveModules[3].getPosition().angle.getDegrees()
            });
            SmartDashboard.putNumberArray("modules pose meters", new double[] {
                    swerveModules[0].getPosition().distanceMeters,
                    swerveModules[1].getPosition().distanceMeters,
                    swerveModules[2].getPosition().distanceMeters,
                    swerveModules[3].getPosition().distanceMeters
            });

            SmartDashboard.putNumberArray("Virtual abs encoders", new double[] {
                    swerveModules[0].getTurningAbsoluteEncoder().getVirtualPosition(),
                    swerveModules[1].getTurningAbsoluteEncoder().getVirtualPosition(),
                    swerveModules[2].getTurningAbsoluteEncoder().getVirtualPosition(),
                    swerveModules[3].getTurningAbsoluteEncoder().getVirtualPosition()
            });
            SmartDashboard.putNumberArray("Raw abs encoders", new double[] {
                    swerveModules[0].getTurningAbsoluteEncoder().getPosition(),
                    swerveModules[1].getTurningAbsoluteEncoder().getPosition(),
                    swerveModules[2].getTurningAbsoluteEncoder().getPosition(),
                    swerveModules[3].getTurningAbsoluteEncoder().getPosition()
            });
            // SmartDashboard.putData("NAVX", m_gyro);

            SmartDashboard.putData("NAVX", m_gyro);
            SmartDashboard.putNumber("Angle", getGyroAngle());
        }

        else {

            SmartDashboard.putNumber("Fake Gyro value:", getGyroAngle());
            SmartDashboard.putData("Odometry Pose Field", field);

            // SmartDashboard.putData("Debugging field",
            // VisionSubsystem.visionSim.getDebugField());

            publisher.set(new SwerveModuleState[] { swerveModuleSims[0].getState(),
                    swerveModuleSims[1].getState(),
                    swerveModuleSims[2].getState(), swerveModuleSims[3].getState() });

        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro", getGyroAngle());
    }

    private void updateOdometry() {
        // Update the odometry (Called in periodic)

        if (RobotBase.isSimulation()) {

            m_trackedRotation = m_trackedRotation.plus(new Rotation2d(
                    DrivetrainConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                            * SwerveModuleSim.getPeriodicRate()));
            fakeGyro = m_trackedRotation.getDegrees();

            m_odometry.update(
                    Rotation2d.fromDegrees(fakeGyro),
                    new SwerveModulePosition[] {
                            swerveModuleSims[0].getPosition(),
                            swerveModuleSims[1].getPosition(),
                            swerveModuleSims[2].getPosition(),
                            swerveModuleSims[3].getPosition()
                    });
        } else {
            m_trackedRotation = new Rotation2d(getGyroAngle());
            m_odometry.update(
                    Rotation2d.fromDegrees(DrivetrainConstants.GYRO_ORIENTATION * m_gyro.getAngle()),
                    new SwerveModulePosition[] {
                            swerveModules[0].getPosition(),
                            swerveModules[1].getPosition(),
                            swerveModules[2].getPosition(),
                            swerveModules[3].getPosition()
                    });
        }
        field.setRobotPose(m_odometry.getEstimatedPosition());
        RobotState.updatePose(m_odometry.getEstimatedPosition());
    }

    public static Pose2d getPose() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED ? m_odometry.getEstimatedPosition() : new Pose2d();
    }

    public void resetPose(Pose2d pose) {
        resetOdometry(pose);
    }

    public void resetOdometry(Pose2d pose) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            if (RobotBase.isReal()) {

                m_odometry.resetPosition(
                        Rotation2d.fromDegrees(DrivetrainConstants.GYRO_ORIENTATION * getGyroAngle()),
                        new SwerveModulePosition[] {
                                swerveModules[0].getPosition(),
                                swerveModules[1].getPosition(),
                                swerveModules[2].getPosition(),
                                swerveModules[3].getPosition()
                        },
                        pose);
            } else {
                m_odometry.resetPosition(
                        Rotation2d.fromDegrees(DrivetrainConstants.GYRO_ORIENTATION * getGyroAngle()),
                        new SwerveModulePosition[] {
                                swerveModuleSims[0].getPosition(),
                                swerveModuleSims[1].getPosition(),
                                swerveModuleSims[2].getPosition(),
                                swerveModuleSims[3].getPosition()
                        },
                        pose);
            }
        }
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            double xSpeedCommanded;
            double ySpeedCommanded;

            if (rateLimit) {
                // Convert XY to polar for rate limiting
                double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
                double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

                // Calculate the direction slew rate based on an estimate of the lateral
                // acceleration
                double directionSlewRate;

                if (m_currentTranslationMag != 0.0) {
                    directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
                } else {
                    directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
                }

                double currentTime = WPIUtilJNI.now() * 1e-6;
                double elapsedTime = currentTime - m_prevTime;
                double angleDif = SwerveUtils.angleDifference(inputTranslationDir, m_currentTranslationDir);

                if (angleDif < 0.45 * Math.PI) {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                } else if (angleDif > 0.85 * Math.PI) {
                    if (m_currentTranslationMag > 1e-4) {
                        m_currentTranslationMag = m_magLimiter.calculate(0.0);
                    } else {
                        m_currentTranslationDir = SwerveUtils.wrapAngle(m_currentTranslationDir + Math.PI);
                        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                    }
                } else {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                }

                m_prevTime = currentTime;

                xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
                ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
                m_currentRotation = m_rotLimiter.calculate(rot);

            } else {
                xSpeedCommanded = xSpeed;
                ySpeedCommanded = ySpeed;
                m_currentRotation = rot;
            }

            // Convert the commanded speeds into the correct units for the drivetrain
            double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

            Rotation2d rotation = Rotation2d.fromDegrees(
                    getGyroAngle()
                            + (GforceUtils.isRedAlliance() ? 180 : 0));

            var swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                    fieldRelative
                            ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation)
                            : speeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(
                    swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
            if (UserPolicy.isManualControlled) {
                if (RobotBase.isReal()) {
                    swerveModules[0].setDesiredState(swerveModuleStates[0]);
                    swerveModules[1].setDesiredState(swerveModuleStates[1]);
                    swerveModules[2].setDesiredState(swerveModuleStates[2]);
                    swerveModules[3].setDesiredState(swerveModuleStates[3]);
                } else {
                    swerveModuleSims[0].setDesiredState(swerveModuleStates[0]);
                    swerveModuleSims[1].setDesiredState(swerveModuleStates[1]);
                    swerveModuleSims[2].setDesiredState(swerveModuleStates[2]);
                    swerveModuleSims[3].setDesiredState(swerveModuleStates[3]);
                }
            }
        }
    }

    public void runChassisSpeeds(ChassisSpeeds speeds, Boolean fieldRelative) {
        Rotation2d rotation = RobotBase.isSimulation() ? Rotation2d.fromDegrees(
                getGyroAngle()
                        + (GforceUtils.isRedAlliance() ? 180 : 0))
                : Rotation2d.fromDegrees(0);

        var swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, rotation)
                        : speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);

        if (RobotBase.isReal()) {
            swerveModules[0].setDesiredState(swerveModuleStates[0]);
            swerveModules[1].setDesiredState(swerveModuleStates[1]);
            swerveModules[2].setDesiredState(swerveModuleStates[2]);
            swerveModules[3].setDesiredState(swerveModuleStates[3]);
        } else {
            swerveModuleSims[0].setDesiredState(swerveModuleStates[0]);
            swerveModuleSims[1].setDesiredState(swerveModuleStates[1]);
            swerveModuleSims[2].setDesiredState(swerveModuleStates[2]);
            swerveModuleSims[3].setDesiredState(swerveModuleStates[3]);
        }
    }

    public ChassisSpeeds getTheoreticalSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative,
            boolean rateLimit) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            double xSpeedCommanded;
            double ySpeedCommanded;

            if (rateLimit) {
                // Convert XY to polar for rate limiting
                double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
                double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

                // Calculate the direction slew rate based on an estimate of the lateral
                // acceleration
                double directionSlewRate;

                if (m_currentTranslationMag != 0.0) {
                    directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
                } else {
                    directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
                }

                double currentTime = WPIUtilJNI.now() * 1e-6;
                double elapsedTime = currentTime - m_prevTime;
                double angleDif = SwerveUtils.angleDifference(inputTranslationDir, m_currentTranslationDir);

                if (angleDif < 0.45 * Math.PI) {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                } else if (angleDif > 0.85 * Math.PI) {
                    if (m_currentTranslationMag > 1e-4) {
                        m_currentTranslationMag = m_magLimiter.calculate(0.0);
                    } else {
                        m_currentTranslationDir = SwerveUtils.wrapAngle(m_currentTranslationDir + Math.PI);
                        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                    }
                } else {
                    m_currentTranslationDir = SwerveUtils.stepTowardsCircular(m_currentTranslationDir,
                            inputTranslationDir,
                            directionSlewRate * elapsedTime);
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                }

                m_prevTime = currentTime;

                xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
                ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
                m_currentRotation = m_rotLimiter.calculate(rot);

            } else {
                xSpeedCommanded = xSpeed;
                ySpeedCommanded = ySpeed;
                m_currentRotation = rot;
            }

            // Convert the commanded speeds into the correct units for the drivetrain
            double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND;
            double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND;

            Rotation2d rotation = Rotation2d.fromDegrees(
                    getGyroAngle()
                            + (GforceUtils.isRedAlliance() ? 180 : 0));

            return !fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                            rotation)
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

        }
        return new ChassisSpeeds();
    }

    // YENİ EKLENDİ: AprilTag'e göre pozisyonlanma komutu
    public Command alignToAprilTag(Pose2d tagPose, double targetDistance) {
        return Commands.run(() -> {
            if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
                Pose2d currentPose = getPose();

                // Hedef pozisyonu hesapla (tag'in önünde belirli mesafede)
                double targetX = tagPose.getX() - targetDistance * Math.cos(tagPose.getRotation().getRadians());
                double targetY = tagPose.getY() - targetDistance * Math.sin(tagPose.getRotation().getRadians());
                Pose2d targetPose = new Pose2d(targetX, targetY, tagPose.getRotation());

                // PID çıktılarını hesapla
                double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
                double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
                double rotSpeed = rotationController.calculate(
                        currentPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());

                // Hızları sınırla
                xSpeed = MathUtil.clamp(xSpeed, -DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * 0.5,
                        DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * 0.5);
                ySpeed = MathUtil.clamp(ySpeed, -DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * 0.5,
                        DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * 0.5);
                rotSpeed = MathUtil.clamp(rotSpeed, -DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.5,
                        DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * 0.5);

                // Robot'u hareket ettir
                ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
                pathFollowDrive(speeds);
            }
        }).until(() -> isAtTargetPose());
    }

    // check if robots in align to target
    private boolean isAtTargetPose() {
        return xController.atSetpoint() &&
                yController.atSetpoint() &&
                rotationController.atSetpoint();
    }

    // getting pose updates from vision
    public void addAprilTagMeasurement(Pose2d visionPose, double timestamp) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            m_odometry.addVisionMeasurement(visionPose, timestamp);
        }
    }

    // added pid values to dashboard
    private void putPIDSmartDashboardData() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("AprilTag/X Error", xController.getPositionError());
            SmartDashboard.putNumber("AprilTag/Y Error", yController.getPositionError());
            SmartDashboard.putNumber("AprilTag/Rotation Error", rotationController.getPositionError());
        }
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            if (RobotBase.isReal()) {
                swerveModules[0].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(45)));
                swerveModules[1].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(-45)));
                swerveModules[2].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(-45)));
                swerveModules[3].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(45)));
            } else {
                swerveModuleSims[0].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(45)));
                swerveModuleSims[1].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(-45)));
                swerveModuleSims[2].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(-45)));
                swerveModuleSims[3].setDesiredState(new SwerveModuleState(0,
                        Rotation2d.fromDegrees(45)));
            }
        }
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    desiredStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND);
            if (RobotBase.isReal()) {
                swerveModules[0].setDesiredState(desiredStates[0]);
                swerveModules[1].setDesiredState(desiredStates[1]);
                swerveModules[2].setDesiredState(desiredStates[2]);
                swerveModules[3].setDesiredState(desiredStates[3]);
            } else {
                swerveModuleSims[0].setDesiredState(desiredStates[0]);
                swerveModuleSims[1].setDesiredState(desiredStates[1]);
                swerveModuleSims[2].setDesiredState(desiredStates[2]);
                swerveModuleSims[3].setDesiredState(desiredStates[3]);
            }
        }
    }

    /**
     * Resets the drive encoders to currently read a position of 0 and sets the
     * turn encoders using the absolute encoders.
     */
    public void resetEncoders() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            swerveModules[0].resetEncoders();
            swerveModules[2].resetEncoders();
            swerveModules[1].resetEncoders();
            swerveModules[3].resetEncoders();
        }
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        if (SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED) {
            if (RobotBase.isReal()) {
                m_gyro.reset();
            } else {
                fakeGyro = 0;
            }

        }
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Double getHeading() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED // if this is true
                ? (Rotation2d.fromDegrees(DrivetrainConstants.GYRO_ORIENTATION * getGyroAngle()).getDegrees())
                : 0;
    }

    public Optional<SwerveModule> getFrontLeftModule() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED ? Optional.of(swerveModules[0]) : Optional.empty();
    }

    public Optional<SwerveModule> getFrontRightModule() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED ? Optional.of(swerveModules[1]) : Optional.empty();
    }

    public Optional<SwerveModule> getRearLeftModule() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED ? Optional.of(swerveModules[2]) : Optional.empty();
    }

    public Optional<SwerveModule> getRearRightModule() {
        return SubsystemEnabledConstants.DRIVE_SUBSYSTEM_ENABLED ? Optional.of(swerveModules[3]) : Optional.empty();
    }

    public void pathFollowDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }

    private ChassisSpeeds getChassisSpeeds() {

        return RobotBase.isReal()
                ? ChassisSpeeds.fromRobotRelativeSpeeds(m_gyro.getVelocityX(), m_gyro.getVelocityY(),
                        Units.degreesToRadians(m_gyro.getRate()),
                        m_gyro.getRotation2d())
                : ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, new Rotation2d());
    }

    public Command gyroReset() {
        return Commands.startEnd(() -> {
            // init
            if (RobotBase.isReal()) {
                zeroHeading();
            }
        }, () -> {
            // end
        });
    }

    public Command xCommand() {
        return Commands.startEnd(() -> {
            // init
            UserPolicy.xLocked = !UserPolicy.xLocked;
        }, () -> {
            // end
        });

    }

    public double turnPID() {
        double output = (turningPID.calculate(LimelightHelpers.getTX(RobotConstants.LLName), 0));
        SmartDashboard.putNumber("turningoutput", output);
        return output;
    }

    public void aimWhileMovingv2(double PIDValue) {
        var speeds = new ChassisSpeeds(
                ((driveJoystick.getRawAxis(3)) * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND),
                ((driveJoystick.getRawAxis(3)) * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND), PIDValue);
        // apply swerve module states
        setModuleStates(DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds));
    }

}

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotConstants.PortConstants.CAN;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ShooterSubsystem extends SubsystemBase {
    SparkMax shooterMotor1;
    SparkMaxConfig shooterMotor1Config;
    static SparkClosedLoopController shooterMotor1Controller;
    private DigitalInput limitSwitch;
    private boolean isShooterRunning = false;
    private boolean gameElementDetected = false;
    private PWMSparkMax ledController;
    private static final double RED = 0.61;
    private static final double GREEN = 0.77;

    public ShooterSubsystem() {
        limitSwitch = new DigitalInput(ShooterConstants.LIMIT_PORT); // Updated name to reflect limit switch
        shooterMotor1 = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
        shooterMotor1Controller = shooterMotor1.getClosedLoopController();
        ledController = new PWMSparkMax(0);

        // Motor configurations
        shooterMotor1Config = new SparkMaxConfig();
        /** shooterMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterMotor1Config.closedLoop.maxMotion.maxVelocity(ShooterConstants.MAX_MOTOR_RPM);
        shooterMotor1Config.closedLoop.maxMotion.maxAcceleration(ShooterConstants.MAX_MOTOR_ACCELERATION);
        shooterMotor1Config.closedLoop.pid(.5, 0.0, 0.0); */
    }

    @Override
    public void periodic() {
        // Update game element detection status
        gameElementDetected = !limitSwitch.get(); // Inverted for normally open switch

        // When game element is detected
        if (gameElementDetected) {
            stopShooter();
        }

        updateLEDStatus();

        // Debug information
        SmartDashboard.putBoolean("Limit Switch Status", limitSwitch.get());
        SmartDashboard.putBoolean("Is Shooter Running", isShooterRunning);
        SmartDashboard.putBoolean("Game Element Detected", gameElementDetected);
    }

    private void updateLEDStatus() {
        if (gameElementDetected) {
            ledController.set(GREEN); // Game element ready
        } else {
            ledController.set(RED); // Waiting for game element
        }
    }

    public void shooterButton() {
        if (!gameElementDetected) {
            // If no game element and motor is not running, start motor
            moveAtSpeed(0.5); // Collect at half speed
        } else {
            // If motor is already running, stop it
            stopShooter();
        }
    }

    public void moveAtSpeed(double speed) {
        shooterMotor1.set(speed);
        isShooterRunning = true;
    }

    public void stopShooter() {
        shooterMotor1.set(0);
        isShooterRunning = false;
    }

    public boolean isShooterRunning() {
        return isShooterRunning;
    }

    public boolean hasGameElement() {
        return gameElementDetected;
    }
}
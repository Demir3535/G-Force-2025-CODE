package frc.robot.subsystems.shooter;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ShooterConstants;
import frc.robot.RobotConstants.PortConstants.CAN;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
public class ShooterSubsystem extends SubsystemBase {
    private SparkMax shooterMotor;
    private DigitalInput limitSwitch;
    private boolean isShooterRunning = false;
    private boolean gameElementDetected = false;
    private PWMSparkMax ledController;
    private static final double RED = 0.61;
    private static final double GREEN = 0.77;
    
    private boolean readyToShoot = false; // Is game piece ready?
    public ShooterSubsystem() {
        limitSwitch = new DigitalInput(ShooterConstants.LIMIT_PORT);
        shooterMotor = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
        ledController = new PWMSparkMax(0);
    }
    @Override
    public void periodic() {
        // Check limit switch status
        gameElementDetected = !limitSwitch.get();
        
        // If game piece is detected and motor is in intake mode, stop the motor
        if (gameElementDetected && isShooterRunning && !readyToShoot) {
            stopShooter();
            readyToShoot = true; // Ready to shoot
        }
        updateLEDStatus();
        // Debug information
        SmartDashboard.putBoolean("Limit Switch Status", limitSwitch.get());
        SmartDashboard.putBoolean("Is Shooter Running", isShooterRunning);
        SmartDashboard.putBoolean("Game Element Detected", gameElementDetected);
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot);
    }
    private void updateLEDStatus() {
        if (readyToShoot) {
            ledController.set(GREEN); // Ready to shoot
        } else {
            ledController.set(RED); // Waiting for game piece
        }
    }
    public void shooterButton() {
        if (!gameElementDetected && !isShooterRunning) {
            // If no game piece and motor is stopped -> Start intake
            moveAtSpeed(0.5); // Half power for intake 
            readyToShoot = false;
        }
        else if (readyToShoot && gameElementDetected) {
            // If game piece present and ready to shoot -> Shoot
            moveAtSpeed(1.0); // Full power for shooting
            readyToShoot = false;
        }
        else if (isShooterRunning) {
            // If motor is running -> Stop
            stopShooter();
        }
    }
    public void moveAtSpeed(double speed) {
        shooterMotor.set(speed);
        isShooterRunning = true;
    }
    public void stopShooter() {
        shooterMotor.set(0);
        isShooterRunning = false;
    }
    public boolean isShooterRunning() {
        return isShooterRunning;
    }
    public boolean hasGameElement() {
        return gameElementDetected;
    }
}
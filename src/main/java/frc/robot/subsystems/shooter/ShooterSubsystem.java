package frc.robot.subsystems.shooter;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.PortConstants.DIO;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
public class ShooterSubsystem extends SubsystemBase {
    private SparkMax shooterMotor;
    private DigitalInput limitSwitch;
    private boolean isShooterRunning = false;
    private boolean gameElementDetected = false;
    private PWMSparkMax ledController;
    private static final double RED = 0.61;
    private static final double GREEN = 0.77;
    private boolean isShooting = false;
    private boolean readyToShoot = false; // Is game piece ready?
    public ShooterSubsystem() {
        limitSwitch = new DigitalInput(DIO.SHOOTER_DISTANCE_SENSOR);
        shooterMotor = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
        ledController = new PWMSparkMax(0);
    }

    @Override
    public void periodic() {
        // Check limit switch status
        gameElementDetected = limitSwitch.get();
        
        // If game piece is detected and motor is in intake mode, stop the motor
   if (isShooting && !gameElementDetected) {
    stopShooter();
    isShooting = false;

   }
   else if (isShooting) {
    
   }
   
        else if (gameElementDetected && isShooterRunning && !readyToShoot) {
            
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
        // Debug info
        SmartDashboard.putBoolean("Button Pressed", true);
        SmartDashboard.putNumber("Motor Output", shooterMotor.get());
        
        if (!gameElementDetected && !isShooterRunning) {
            SmartDashboard.putString("Shooter State", "Starting Intake");
            moveAtSpeed(-0.25);
            readyToShoot = false;
        }
        else if (readyToShoot && gameElementDetected) {
            isShooting = true;
            SmartDashboard.putString("Shooter State", "Shooting");
            moveAtSpeed(-0.25)   ;
            readyToShoot = false;
        }
        else if (isShooterRunning) {
            SmartDashboard.putString("Shooter State", "Stopping");
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
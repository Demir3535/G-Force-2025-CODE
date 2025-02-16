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
import frc.robot.RobotConstants.PortConstants.DIO;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class ShooterSubsystem extends SubsystemBase {
   SparkMax shooterMotor1;
   SparkMax shooterMotor2;
   SparkMaxConfig shooterMotor1Config;
   SparkMaxConfig shooterMotor2Config;
   static SparkClosedLoopController shooterMotor1Controller;
   private DigitalInput distanceSensor;
   private boolean isShooterRunning = false;
   private boolean shootingMode = false; // NEW: Added for shooting mode
   private PWMSparkMax ledController;
   private static final double RED = 0.61;
   private static final double GREEN = 0.77;

   public ShooterSubsystem() {
       distanceSensor = new DigitalInput(DIO.SHOOTER_DISTANCE_SENSOR);
       shooterMotor1 = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
       shooterMotor2 = new SparkMax(CAN.SHOOTER_MOTOR_2, MotorType.kBrushless);
       shooterMotor1Controller = shooterMotor1.getClosedLoopController();
       ledController = new PWMSparkMax(0);

       shooterMotor1Config = new SparkMaxConfig();
       shooterMotor2Config = new SparkMaxConfig();
       shooterMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
       shooterMotor1Config.closedLoop.maxMotion.maxVelocity(ShooterConstants.MAX_MOTOR_RPM);
       shooterMotor1Config.closedLoop.maxMotion.maxAcceleration(ShooterConstants.MAX_MOTOR_ACCELERATION);
       shooterMotor1Config.closedLoop.pid(.5, 0.0, 0.0);
       shooterMotor2Config.follow(CAN.SHOOTER_MOTOR_1, true);

       shooterMotor1.configure(shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       shooterMotor2.configure(shooterMotor2Config, ResetMode.kResetSafeParameters,
               PersistMode.kPersistParameters);
   }

   @Override
   public void periodic() {
       // NEW: Added shooting mode control
       if (!shootingMode && distanceSensor.get()) {
           stopShooter();
           isShooterRunning = false;
       }
       updateLEDStatus();

       // Debug information
       SmartDashboard.putBoolean("Shooter Distance Sensor", distanceSensor.get());
       SmartDashboard.putBoolean("Is Shooter Running", isShooterRunning);
       SmartDashboard.putBoolean("Coral Detected", !distanceSensor.get());
       SmartDashboard.putBoolean("Shooting Mode", shootingMode); // NEW: Added shooting mode status
   }

   private void updateLEDStatus() {
       if (distanceSensor.get()) {
           ledController.set(RED);
       } else {
           ledController.set(GREEN);
       }
   }

   // NEW: Updated moveAtSpeed method
   public void moveAtSpeed(double speed) {
       shootingMode = true;
       shooterMotor1.set(speed * 0.5);
       isShooterRunning = true;
   }

   // NEW: Updated stopShooter method
   public void stopShooter() {
       shooterMotor1.set(0);
       isShooterRunning = false;
       shootingMode = false; // Turn off shooting mode
   }

   public boolean isShooterRunning() {
       return isShooterRunning;
   }

   // NEW: New method to check shooting mode status
   public boolean isShootingMode() {
       return shootingMode;
   }
}
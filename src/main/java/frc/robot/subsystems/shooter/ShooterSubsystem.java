package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.ShooterConstans;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.PortConstants.DIO;
import frc.robot.RobotConstants.PortConstants.PWM;
import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.Robot;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;

public class ShooterSubsystem extends SubsystemBase {
   SparkMax shooterMotor1;
   SparkMax shooterMotor2;
   SparkMaxConfig shooterMotor1Config;
   SparkMaxConfig shooterMotor2Config;
   static SparkClosedLoopController shooterMotor1Controller;

   private DigitalInput distanceSensor;
   private boolean isShooterRunning = false;
   private boolean shootingMode = false; // YENİ: Atış modu için eklendi

   private PWMMotorController ledController;
   
   private static final double RED = 0.61;
   private static final double GREEN = 0.77;

   public ShooterSubsystem() {
       distanceSensor = new DigitalInput(DIO.SHOOTER_DISTANCE_SENSOR);

       shooterMotor1 = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
       shooterMotor2 = new SparkMax(CAN.SHOOTER_MOTOR_2, MotorType.kBrushless);
    
       shooterMotor1Controller = shooterMotor1.getClosedLoopController();

       shooterMotor1Config = new SparkMaxConfig();
       shooterMotor2Config = new SparkMaxConfig();

       shooterMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
       shooterMotor1Config.closedLoop.maxMotion.maxVelocity(ShooterConstans.MAX_MOTOR_RPM);
       shooterMotor1Config.closedLoop.maxMotion.maxAcceleration(ShooterConstans.MAX_MOTOR_ACCELERATION);
       shooterMotor1Config.closedLoop.pid(.5, 0.0, 0.0);
       shooterMotor2Config.follow(CAN.SHOOTER_MOTOR_1, true);

       shooterMotor1.configure(shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       shooterMotor2.configure(shooterMotor2Config, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
   }

   @Override
   public void periodic() {
       // YENİ: Atış modu kontrolü eklendi
       if (!shootingMode && distanceSensor.get()) {
           stopShooter();
           isShooterRunning = false;
       }

       //updateLEDStatus();

       // Debug bilgileri
       SmartDashboard.putBoolean("Shooter Distance Sensor", distanceSensor.get());
       SmartDashboard.putBoolean("Is Shooter Running", isShooterRunning);
       SmartDashboard.putBoolean("Coral Detected", !distanceSensor.get());
       SmartDashboard.putBoolean("Shooting Mode", shootingMode); // YENİ: Atış modu durumu eklendi
   }

   private void updateLEDStatus() {
       if (distanceSensor.get()) {
           ledController.set(RED);
       } else {
           ledController.set(GREEN);
       }
   }

   // YENİ: moveAtSpeed metodu güncellendi
   public void moveAtSpeed(double speed) {
       if (shootingMode || !distanceSensor.get()) {
           shooterMotor1.set(speed * .5);
           isShooterRunning = true;
           shootingMode = true; // Atış modunu aktif et
       }
   }

   // YENİ: stopShooter metodu güncellendi
   public void stopShooter() {
       shooterMotor1.set(0);
       isShooterRunning = false;
       shootingMode = false; // Atış modunu kapat
   }

   public boolean isShooterRunning() {
       return isShooterRunning;
   }

   // YENİ: Atış modu durumunu kontrol etmek için yeni metod
   public boolean isShootingMode() {
       return shootingMode;
   }
}
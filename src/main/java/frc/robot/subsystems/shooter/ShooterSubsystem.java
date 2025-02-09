package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput; // Rev 2m Distance Sensor için DigitalInput ekledim
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
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.PortConstants.DIO; // DIO (Digital Input/Output) portları için ekledim

import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.Robot;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ShooterSubsystem extends SubsystemBase {
    SparkMax shooterMotor1;
    SparkMax shooterMotor2;
    SparkMaxConfig shooterMotor1Config;
    SparkMaxConfig shooterMotor2Config;
    static SparkClosedLoopController shooterMotor1Controller;

    private DigitalInput distanceSensor; // Rev 2m Distance Sensor için DigitalInput tanımladım
    private boolean isShooterRunning = false; // Shooter'ın çalışıp çalışmadığını takip etmek için boolean değişken

    public ShooterSubsystem() {
        // Rev 2m Distance Sensor'ı DIO portuna bağladım
        distanceSensor = new DigitalInput(DIO.SHOOTER_DISTANCE_SENSOR); // DIO.SHOOTER_DISTANCE_SENSOR, sensörün bağlı olduğu port

        // Motor ve kontrolcü kurulumu
        shooterMotor1 = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
        shooterMotor1Controller = shooterMotor1.getClosedLoopController();

        shooterMotor1Config = new SparkMaxConfig();
        shooterMotor2Config = new SparkMaxConfig();

        shooterMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterMotor1Config.closedLoop.maxMotion.maxVelocity(ShooterConstans.MAX_MOTOR_RPM);
        shooterMotor1Config.closedLoop.maxMotion.maxAcceleration(ShooterConstans.MAX_MOTOR_ACCELERATION);
        shooterMotor1Config.closedLoop.pid(.5, 0.0, 0.0);

        shooterMotor1.configure(shooterMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Sensör durumunu sürekli kontrol ediyoruz
        if (distanceSensor.get()) { // Sensör bir nesne algıladığında (true)
            stopShooter(); // Motorları durdur
            isShooterRunning = false; // Shooter'ın durduğunu işaretle
        }

        // Sensör durumunu SmartDashboard'a yazdır (debug için)
        SmartDashboard.putBoolean("Shooter Distance Sensor", distanceSensor.get());
        SmartDashboard.putBoolean("Is Shooter Running", isShooterRunning);
    }

    public void moveAtSpeed(double speed) {
        if (!distanceSensor.get()) { // Sensör bir nesne algılamıyorsa (false)
            shooterMotor1.set(speed * .5);
            isShooterRunning = true; // Shooter'ın çalıştığını işaretle
        }
    }

    public void stopShooter() {
        shooterMotor1.set(0);
        isShooterRunning = false; // Shooter'ın durduğunu işaretle
    }

    public boolean isShooterRunning() {
        return isShooterRunning; // Shooter'ın çalışıp çalışmadığını döndür
    }
}
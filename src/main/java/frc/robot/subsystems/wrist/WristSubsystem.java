package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.subsystems.ElevatorWristSim;

import com.revrobotics.spark.SparkBase.ResetMode;

@Logged
public class WristSubsystem extends SubsystemBase {
    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig;
    static SparkClosedLoopController wristMotorController;
    private double targetSetpoint = 0;

    public WristSubsystem() {
        wristMotor = new SparkMax(CAN.WRIST_MOTOR, MotorType.kBrushless);

        wristMotorController = wristMotor.getClosedLoopController();

        wristMotorConfig = new SparkMaxConfig();

        wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristMotorConfig.closedLoop.maxMotion.maxVelocity(WristConstants.MAX_MOTOR_RPM);
        wristMotorConfig.closedLoop.maxMotion.maxAcceleration(WristConstants.MAX_MOTOR_ACCELERATION);

        wristMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(.5);

        // PID değerlerini güncelledim (daha iyi hedefte kalma için)
        wristMotorConfig.closedLoop.pid(0.2, 0.001, 0.1); // P, I, D değerleri

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Simülasyon kombinasyonu, RobotContainer'da yapılıyor
    }

    public void goToSetpoint(double setpoint) {
        double limitedSetpoint = limitSetpoint(setpoint);
        this.targetSetpoint = limitedSetpoint;
    
        if (RobotBase.isReal()) {
            double gravityFeedforward = 0.1; // Yerçekimi feedforward'u
            wristMotorController.setReference(limitedSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, gravityFeedforward);
        }
    }
    // Setpoint değerini WristConstants içindeki limitlere göre sınırla
    private double limitSetpoint(double setpoint) {
        if (setpoint > WristConstants.WRIST_MAX_ANGLE) {
            return WristConstants.WRIST_MAX_ANGLE;
        } else if (setpoint < WristConstants.WRIST_MIN_ANGLE) {
            return WristConstants.WRIST_MIN_ANGLE;
        }
        return setpoint;
    }

    public void moveAtSpeed(double speed) {
        // Encoder değeri limitlere ulaştıysa o yönde hareketi durdur
        double currentPosition = getEncoderValue();

        if ((currentPosition >= WristConstants.WRIST_MAX_ANGLE && speed > 0) || 
            (currentPosition <= WristConstants.WRIST_MIN_ANGLE && speed < 0)) {
            wristMotor.set(0);
        } else {
            wristMotor.set(speed * .5);
        }
    }

    public Command goToCoralScoreSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 1) {
                    setpoint = WristConstants.AngleSetpoints.Coral.L1;
                } else if (level == 2) {
                    setpoint = WristConstants.AngleSetpoints.Coral.L2;
                } else if (level == 3) {
                    setpoint = WristConstants.AngleSetpoints.Coral.L3;
                } else {
                    setpoint = WristConstants.AngleSetpoints.HOME;
                }
                goToSetpoint(setpoint);
            } else {
                ElevatorWristSim.goToScoreSetpoint(level);
            }
        }, this);
    }

    public void setEncoderValue(double value) {
        wristMotor.getEncoder().setPosition(value);
    }

    public Command goToAlgaeGrabSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 2) {
                    setpoint = WristConstants.AngleSetpoints.Algae.L2;
                } else if (level == 3) {
                    setpoint = WristConstants.AngleSetpoints.Algae.L3;
                } else {
                    setpoint = WristConstants.AngleSetpoints.HOME;
                }
                goToSetpoint(setpoint);
            }
        }, this);
    }

    public Command goToHumanPlayerSetpoint() {
        return new InstantCommand(() -> {
            if (RobotBase.isReal()) {
                goToSetpoint(WristConstants.AngleSetpoints.HP);
            }
        }, this);
    }

    public double getEncoderValue() {
        return wristMotor.getEncoder().getPosition();
    }

    // Wrist'in hedefte olup olmadığını kontrol eden metodu ekledim
    public boolean atSetpoint() {
        double currentPosition = getEncoderValue();
        double tolerance = 1.0; // Tolerans değeri
        return Math.abs(currentPosition - targetSetpoint) <= tolerance;
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            // SmartDashboard'a teşhis bilgisi ekleyelim
            SmartDashboard.putNumber("Wrist Encoder Position", getEncoderValue());
            SmartDashboard.putNumber("Wrist Target Position", targetSetpoint);
            SmartDashboard.putNumber("Wrist Position Error", getEncoderValue() - targetSetpoint);
            SmartDashboard.putNumber("Wrist Motor Output", wristMotor.getAppliedOutput());
            SmartDashboard.putBoolean("Wrist At Upper Limit", getEncoderValue() >= WristConstants.WRIST_MAX_ANGLE);
            SmartDashboard.putBoolean("Wrist At Lower Limit", getEncoderValue() <= WristConstants.WRIST_MIN_ANGLE);

            // Hedef değerini kontrol et ve gerekirse tekrar gönder
            double currentPos = getEncoderValue();
            if (Math.abs(currentPos - targetSetpoint) > 0.5) {
                wristMotorController.setReference(targetSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
            }

            // Eğer encoder değeri limitlerin ötesindeyse, motoru durdur
            if (currentPos > WristConstants.WRIST_MAX_ANGLE || currentPos < WristConstants.WRIST_MIN_ANGLE) {
                wristMotor.set(0);
            }

            // Eğer wrist hedefteyse, motor çıkışını sıfırla (ekledim)
            if (atSetpoint()) {
                wristMotor.set(0);
            }
        }
    }
}
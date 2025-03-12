package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.RobotConstants.ElevatorConstants;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    SparkMax elevatorMotor1;
    SparkMax elevatorMotor2;
    SparkMaxConfig elevatorMotor1Config;
    SparkMaxConfig elevatorMotor2Config;
    static SparkClosedLoopController elevatorMotor1Controller;
    private double targetSetpoint = 0; // Sınıf seviyesinde değişken
    private static ElevatorSubsystem instance;
    public ElevatorSubsystem() {
        elevatorMotor1 = new SparkMax(CAN.ELEVATOR_MOTOR_1, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(CAN.ELEVATOR_MOTOR_2, MotorType.kBrushless);

        elevatorMotor1Controller = elevatorMotor1.getClosedLoopController();

        elevatorMotor1Config = new SparkMaxConfig();
        elevatorMotor2Config = new SparkMaxConfig();

        elevatorMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorMotor1Config.closedLoop.maxMotion.allowedClosedLoopError(.75);
        elevatorMotor1Config.closedLoop.maxMotion.maxVelocity(ElevatorConstants.MAX_MOTOR_RPM);
        elevatorMotor1Config.closedLoop.maxMotion.maxAcceleration(ElevatorConstants.MAX_MOTOR_ACCELERATION);

        elevatorMotor1Config.closedLoop.pid(0.05, 0.0, 0.0);

        elevatorMotor2Config.follow(CAN.ELEVATOR_MOTOR_1, true);

        elevatorMotor1.configure(elevatorMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // } else {
        new ElevatorWristSim();
    }

    // Setpoint değerini limitlere göre sınırlayan yeni metod
    private double limitSetpoint(double setpoint) {
        if (setpoint < ElevatorConstants.ELEVATOR_MAX_HEIGHT) {
            return ElevatorConstants.ELEVATOR_MAX_HEIGHT;
        } else if (setpoint > ElevatorConstants.ELEVATOR_MIN_HEIGHT) {
            return ElevatorConstants.ELEVATOR_MIN_HEIGHT;
        }
        return setpoint;
    }




    public void goToSetpoint(double setpoint) {
        // Setpoint değerini limitlere göre sınırla
        double limitedSetpoint = limitSetpoint(setpoint);
        this.targetSetpoint = limitedSetpoint;

        // Add code here to move the elevator to the scoring height
        if (RobotBase.isReal()) {
            elevatorMotor1Controller.setReference(limitedSetpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0.2);
        }
    }

    public boolean atSetpoint() {
        double currentPosition = getEncoderValue(); // Elevator'un mevcut pozisyonu
        double tolerance = 1.0; // Tolerans değeri
        return Math.abs(currentPosition - targetSetpoint) <= tolerance;
    }

    public void setEncoderValue(double value) {
        // In rotations
        elevatorMotor1.getEncoder().setPosition(value);
    }

    public double getEncoderValue() {
        return elevatorMotor1.getEncoder().getPosition();
    }

    public void setMotorVoltage(double volts) {
        elevatorMotor1.setVoltage(volts);
    }

    public Command goToCoralScoreSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 1) {
                    setpoint = ElevatorConstants.HeightSetpoints.Coral.L1;
                } else if (level == 2) {
                    setpoint = ElevatorConstants.HeightSetpoints.Coral.L2;
                } else if (level == 3) {
                    setpoint = ElevatorConstants.HeightSetpoints.Coral.L3;
                } else {
                    setpoint = ElevatorConstants.HeightSetpoints.HOME;
                }
                goToSetpoint(setpoint);
            } else {
                ElevatorWristSim.goToScoreSetpoint(level);
            }
        }, this);
    }

    public Command goToHumanPlayerPickup() {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                goToSetpoint(ElevatorConstants.HeightSetpoints.HP);
            } else {
                ElevatorWristSim.setElevatorToHeight(ElevatorConstants.SimConstants.HP);// Passes in the L1-L3 into the sim logic
            }
        }, this);
    }

    public Command goToAlgaeGrabSetpoint(int level) {
        return new InstantCommand(() -> {
            double setpoint;
            if (RobotBase.isReal()) {
                if (level == 2) {
                    setpoint = ElevatorConstants.HeightSetpoints.Algae.L2;
                } else if (level == 3) {
                    setpoint = ElevatorConstants.HeightSetpoints.Algae.L3;
                } else {
                    setpoint = ElevatorConstants.HeightSetpoints.HOME;
                }
                goToSetpoint(setpoint);
            } else {
                //ElevatorWristSim.goToScoreSetpoint(level);// Passes in the L1-L3 into the sim logic, this needs some work.
            }
        }, this);
    }

    public void moveAtSpeed(double speed) {
        // Encoder değeri limitlere ulaştıysa o yönde hareketi durdur
        double currentPosition = elevatorMotor1.getEncoder().getPosition();
        
        if ((currentPosition <= ElevatorConstants.ELEVATOR_MAX_HEIGHT && speed < 0) || 
            (currentPosition >= ElevatorConstants.ELEVATOR_MIN_HEIGHT && speed > 0)) {
            elevatorMotor1.set(0);
        } else {
            elevatorMotor1.set(speed);
        }
    }

    // public Command homeElevator() {
    // return this.run(() -> elevatorMotor1.setVoltage(1)).until(() ->
    // getCurrentDraw() > 30.0)
    // .finallyDo(() -> setEncoderValue(0));
    // }

    public double getCurrentDraw() {
        return elevatorMotor1.getOutputCurrent();
    }

    public RelativeEncoder getEncoder() {
        return elevatorMotor1.getEncoder();
    }

    @Override
    public void periodic() {
        if (RobotBase.isReal()) {
            SmartDashboard.putNumber("elevator encoder pos", elevatorMotor1.getEncoder().getPosition());
            SmartDashboard.putNumber("elevator target pos", targetSetpoint);
            SmartDashboard.putBoolean("Elevator At Max Height", elevatorMotor1.getEncoder().getPosition() <= ElevatorConstants.ELEVATOR_MAX_HEIGHT);
            SmartDashboard.putBoolean("Elevator At Min Height", elevatorMotor1.getEncoder().getPosition() >= ElevatorConstants.ELEVATOR_MIN_HEIGHT);
            
            // Sürekli olarak hedef değerini kontrol et ve gerekirse tekrar gönder
            double currentPos = elevatorMotor1.getEncoder().getPosition();
            if (Math.abs(currentPos - targetSetpoint) > 0.5) {
                elevatorMotor1Controller.setReference(targetSetpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, 0);
            }
            
            // Eğer encoder değeri limitlerin ötesindeyse, motoru durdur
            if (currentPos < ElevatorConstants.ELEVATOR_MAX_HEIGHT || currentPos > ElevatorConstants.ELEVATOR_MIN_HEIGHT) {
                elevatorMotor1.set(0);
            }
        }
    }
}
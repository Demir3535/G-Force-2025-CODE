package frc.robot.subsystems.wrist;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.WristConstants;
import com.revrobotics.spark.SparkBase.ResetMode;

public class WristSubsystem extends SubsystemBase {
    SparkMax wristMotor;
    SparkMaxConfig wristMotorConfig;
    static SparkClosedLoopController wristMotorController;

    public WristSubsystem() {

        // if (RobotBase.isReal()) {
        wristMotor = new SparkMax(CAN.WRIST_MOTOR, MotorType.kBrushless);

        wristMotorController = wristMotor.getClosedLoopController();

        wristMotorConfig = new SparkMaxConfig();

        wristMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        wristMotorConfig.closedLoop.maxMotion.maxVelocity(WristConstants.MAX_MOTOR_RPM);
        wristMotorConfig.closedLoop.maxMotion.maxAcceleration(WristConstants.MAX_MOTOR_ACCELERATION);

        wristMotorConfig.closedLoop.pid(0.1, 0.0, 0.0);

        wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // }
        // The sim combination of wrist and elevator init is done in the RobotContainer
    }

    public static void goToSetpoint(double setpoint) {
        if (RobotBase.isReal()) {
            wristMotorController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
        }

    }

    public void moveAtSpeed(double speed) {
        wristMotor.set(speed * .75);
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
                    setpoint = WristConstants.AngleSetpoints.Coral.L1;
                }
                goToSetpoint(setpoint);
            }

        }, this);
    }
    @Override
    public void periodic() {
        if (RobotBase.isReal()) {

        } else {

        }
    }

}

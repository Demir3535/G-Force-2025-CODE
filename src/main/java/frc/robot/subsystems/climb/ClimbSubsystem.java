package frc.robot.subsystems.climb;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.ClimbConstans;
import frc.robot.RobotConstants.PortConstants.CAN;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ClimbSubsystem extends SubsystemBase {
    SparkMax climbMotor1;

    SparkMaxConfig climbMotor1Config;

    static SparkClosedLoopController climbMotor1Controller;

    public ClimbSubsystem() {

        // if (RobotBase.isReal()) {
        climbMotor1 = new SparkMax(CAN.CLIMB_MOTOR, MotorType.kBrushless);

        climbMotor1Controller = climbMotor1.getClosedLoopController();

        climbMotor1Config = new SparkMaxConfig();

        climbMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        climbMotor1Config.closedLoop.maxMotion.maxVelocity(ClimbConstans.MAX_MOTOR_RPM);
        climbMotor1Config.closedLoop.maxMotion.maxAcceleration(ClimbConstans.MAX_MOTOR_ACCELERATION);



        climbMotor1.configure(climbMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // } else {

    }

    public void moveAtSpeed(double speed) {
        climbMotor1.set(speed * .5);
    }

    public void stopClimb() {
        climbMotor1.set(0);
    }
}
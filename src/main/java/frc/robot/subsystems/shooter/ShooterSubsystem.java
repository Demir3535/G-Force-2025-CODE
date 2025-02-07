package frc.robot.subsystems.shooter;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;


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

import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.Robot;
import com.revrobotics.spark.SparkBase.ResetMode;



public class ShooterSubsystem extends SubsystemBase {
    SparkMax shooterMotor1;
    SparkMax shooterMotor2;
    SparkMaxConfig shooterMotor1Config;
    SparkMaxConfig shooterMotor2Config;
    static SparkClosedLoopController shooterMotor1Controller;

     public ShooterSubsystem() {

        // if (RobotBase.isReal()) {
            shooterMotor1 = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
            

        shooterMotor1Controller = shooterMotor1.getClosedLoopController();
        

        shooterMotor1Config = new SparkMaxConfig();
        shooterMotor2Config = new SparkMaxConfig();

        shooterMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        shooterMotor1Config.closedLoop.maxMotion.maxVelocity(ShooterConstans.MAX_MOTOR_RPM);
        shooterMotor1Config.closedLoop.maxMotion.maxAcceleration(ShooterConstans.MAX_MOTOR_ACCELERATION);
        

        shooterMotor1Config.closedLoop.pid(.5, 0.0, 0.0);
        

        

        shooterMotor1.configure(shooterMotor1Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

               
       // } else {

 }

 public void moveAtSpeed(double speed) {
    shooterMotor1.set(speed * .5);
}

public void stopShooter() {
    shooterMotor1.set(0);
     }
}
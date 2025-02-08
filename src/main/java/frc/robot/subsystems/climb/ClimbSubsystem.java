package frc.robot.subsystems.climb;
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
import frc.robot.RobotConstants.ClimbConstans;
import frc.robot.subsystems.ElevatorWristSim;
import frc.robot.Robot;
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
        climbMotor1Config.closedLoop.maxMotion.maxVelocity(ShooterConstans.MAX_MOTOR_RPM);
        climbMotor1Config.closedLoop.maxMotion.maxAcceleration(ShooterConstans.MAX_MOTOR_ACCELERATION);
        

        climbMotor1Config.closedLoop.pid(.5, 0.0, 0.0);
        

        

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
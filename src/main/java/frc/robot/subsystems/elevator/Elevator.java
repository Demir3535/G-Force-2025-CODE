package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants.PortConstants;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.MotorSpeeds;



public class Elevator extends SubsystemBase {

    PIDController ElevatorPID = new PIDController(MotorSpeeds.eP, MotorSpeeds.eI, MotorSpeeds.eD);
    PIDController ElevatorDownPID = new PIDController(MotorSpeeds.edP, MotorSpeeds.edI, MotorSpeeds.edD);


    //motors & variables here, define them and create any PIDs needed
    private SparkMax elevatorSparkMax;
    private RelativeEncoder elevatorRelativeEncoder;
    private SparkLimitSwitch elevatorBottomLimit;
    SparkMax elevatorMotor1;
    SparkMax elevatorMotor2;
    SparkMaxConfig elevatorMotor1Config;
    SparkMaxConfig elevatorMotor2Config;
    static SparkClosedLoopController elevatorMotor1Controller;

    //private SparkLimitSwitch elevatorTopLimit;

    public Elevator(){
        //config motor settings here
        //config motor settings here
        elevatorMotor1 = new SparkMax(CAN.ELEVATOR_MOTOR_1, MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(CAN.ELEVATOR_MOTOR_2, MotorType.kBrushless);

        elevatorBottomLimit = elevatorSparkMax.getForwardLimitSwitch();
        //elevatorTopLimit = elevatorSparkMax.getForwardLimitSwitch();
        elevatorMotor2Config.follow(CAN.ELEVATOR_MOTOR_1, true);

        //elevatorRelativeEncoder.
        elevatorRelativeEncoder = elevatorSparkMax.getEncoder();
    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
        SmartDashboard.putBoolean("elevatorbottom", elevatorAtBottomLimit());
        SmartDashboard.putNumber("elevatorencoder", elevatorEncoderGet());
        if(elevatorBottomLimit.isPressed()){
            elevatorRelativeEncoder.setPosition(0);
        }

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        //Mostly used for debug and such
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    // Should include run/stop/run back, etc.

    //as well as check for limits and reset encoders,
    //return true/false if limit is true, or encoder >= x value

    public void elevatorUp(){
        elevatorMotor1.set(-MotorSpeeds.elevatorSpeed);
    }

    public void elevatorDown(){
        elevatorMotor1.set(MotorSpeeds.elevatorSpeed);
    }

    //elevator setpoints
    public void elevatorL(double setpoint){
        double calc = ElevatorPID.calculate(elevatorRelativeEncoder.getPosition(), setpoint);
        elevatorMotor1.set(calc);
    }

    public void elevatorStop(){
        double elevatorpos = elevatorRelativeEncoder.getPosition();
        if(elevatorpos <= -2){
            elevatorMotor1.set(MotorSpeeds.elevatorSpeedDown);
        }
        else{
            elevatorMotor1.stopMotor();
        }
    }

    public boolean elevatorAtBottomLimit(){
        return elevatorBottomLimit.isPressed();
    }

    public boolean elevatorAtTopLimit(){
        //return elevatorTopLimit.isPressed();
        return false; 
    }

    public double elevatorEncoderGet(){
        return elevatorRelativeEncoder.getPosition();
    }
}
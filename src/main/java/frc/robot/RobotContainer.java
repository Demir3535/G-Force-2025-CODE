// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.claw.SetClawSpeed;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.MoveElevatorManual;
import frc.robot.commands.wrist.MoveWristManual;
import frc.robot.subsystems.claw.ClawSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.automation.AutomationSelector;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.automation.AutomatedScoring;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.commands.elevator.SetElevatorSpeed;
import frc.robot.RobotConstants.ClimbConstans;
public class RobotContainer {

    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    public final ClawSubsystem clawSubsystem = new ClawSubsystem();
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();


    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
    private final Joystick operatorJoystick = new Joystick(RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);

    public final AutomationSelector automationSelector = new AutomationSelector();

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

    PowerDistribution pdp;

    private final Field2d field = new Field2d();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(new TeleopDriveCommand(driveSubsystem, driveJoystick));

        elevatorSubsystem.setDefaultCommand(new MoveElevatorManual(elevatorSubsystem, operatorJoystick));
        wristSubsystem.setDefaultCommand(new MoveWristManual(wristSubsystem, operatorJoystick));

        createNamedCommands();

        configureButtonBindings();

        try {
            pdp = new PowerDistribution(CAN.PDH, ModuleType.kRev);
            m_autoPositionChooser = AutoBuilder.buildAutoChooser("Test Auto");
            Shuffleboard.getTab("Autonomous Selection").add(m_autoPositionChooser);
            Shuffleboard.getTab("Power").add(pdp);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void createNamedCommands() {
        // Add commands here to be able to execute in auto through pathplanner

        NamedCommands.registerCommand("Example", new RunCommand(() -> {
            System.out.println("Running...");
        }));
        NamedCommands.registerCommand("Score L1",
                AutomatedScoring.scoreNoPathing(1, elevatorSubsystem, wristSubsystem, clawSubsystem));
        NamedCommands.registerCommand("Score L2",
                AutomatedScoring.scoreNoPathing(2, elevatorSubsystem, wristSubsystem, clawSubsystem));
        NamedCommands.registerCommand("Score L3",
                AutomatedScoring.scoreNoPathing(3, elevatorSubsystem, wristSubsystem, clawSubsystem));
    }

    private void configureButtonBindings() {
        

        new JoystickButton(driveJoystick, 9).onChange(driveSubsystem.xCommand()); // Needs to be while true so the
                                                                                  // command ends
        new JoystickButton(driveJoystick, 2).whileTrue(driveSubsystem.gyroReset());

        // new JoystickButton(driveJoystick, 2).whileTrue(
        //         new InstantCommand(() -> {
        //             // Create a new command instance at the time of button press,
        //             // ensuring that the latest values are used.
        //             Command cmd = AutomatedScoring.fullScore(
        //                     automationSelector.getReefSide(),
        //                     automationSelector.getPosition(),
        //                     automationSelector.getHeight(),
        //                     driveSubsystem, elevatorSubsystem, wristSubsystem, clawSubsystem);
        //             cmd.schedule();
        //         }));

        // new JoystickButton(driveJoystick, 6).whileTrue(
        //         new InstantCommand(() -> {
        //             // Create a new command instance at the time of button press,
        //             // ensuring that the latest values are used.
        //             Command cmd = AutomatedScoring.humanPlayerPickup(automationSelector.getHumanPlayerStation(),
        //                     driveSubsystem, elevatorSubsystem, wristSubsystem, clawSubsystem);
        //             cmd.schedule();
        //         }));

        // Above = DriveJoystick, Below = OperatorJoystick
        
        new JoystickButton(operatorJoystick, 2).whileTrue(new SetClawSpeed(clawSubsystem, 0.75));
        new JoystickButton(operatorJoystick, 2).whileTrue(new SetClawSpeed(clawSubsystem, -.75));
        
        new JoystickButton(operatorJoystick, 2).whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.75));
        new JoystickButton(operatorJoystick, 2).whileTrue(new SetElevatorSpeed(elevatorSubsystem, -.75));
        
        final Trigger setClawSpeedButton = new JoystickButton(operatorJoystick, PS4Controller.Button.kSquare.value);
        setClawSpeedButton.whileTrue(new SetClawSpeed(clawSubsystem, .75));
        setClawSpeedButton.whileFalse(new SetClawSpeed(clawSubsystem, 0));


        new JoystickButton(operatorJoystick, 1).whileTrue(elevatorSubsystem.goToScoreSetpoint(1));
        new JoystickButton(operatorJoystick, 6).whileTrue(elevatorSubsystem.goToScoreSetpoint(1));
        new JoystickButton(operatorJoystick, 4).whileTrue(new RunCommand(() -> shooterSubsystem.moveAtSpeed(1.0), shooterSubsystem))
        .onFalse(new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem));
       
        new JoystickButton(operatorJoystick, 4).whileTrue(new RunCommand(() -> climbSubsystem.moveAtSpeed(1.0), climbSubsystem))
        .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb(), climbSubsystem));
       
       
        final Trigger setElevatorSpeedButton = new JoystickButton(operatorJoystick, PS4Controller.Button.kCircle.value);
        setElevatorSpeedButton.whileTrue(new SetElevatorSpeed(elevatorSubsystem, 0.75));
        setElevatorSpeedButton.whileFalse(new SetElevatorSpeed(elevatorSubsystem, 0));
    }

    public Command getAutonomousCommand() {
        if (m_autoPositionChooser.getSelected() != null) {
            return m_autoPositionChooser.getSelected();
        } else {
            return driveSubsystem.gyroReset();
        }
    }

    public Command getTestingCommand() {
        return new RobotSystemsCheckCommand(driveSubsystem);
    }

    public Field2d getField() {
        return field;
    }
    public static final class UserPolicy {
        public static boolean xLocked = false;
        public static boolean isManualControlled = true;
    }
}
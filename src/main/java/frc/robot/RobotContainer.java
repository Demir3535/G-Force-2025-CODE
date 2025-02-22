// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drive.AutoPositionToTagCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.MoveElevatorManual;
import frc.robot.commands.wrist.MoveWristManual;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.automation.AutomationSelector;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.automation.AutomatedScoring;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.drive.AutoPositionToTagCommand;
import frc.robot.commands.drive.LimelightDriveCommand;

public class RobotContainer {

    private final DriveSubsystem m_drive = new DriveSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();

    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
    private final Joystick operatorJoystick = new Joystick(RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);
    public final AutomationSelector automationSelector = new AutomationSelector();

    SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

    PowerDistribution pdp;

    private final Field2d field = new Field2d();

    public RobotContainer() {
        m_drive.setDefaultCommand(new TeleopDriveCommand(m_drive, driveJoystick));

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
                AutomatedScoring.scoreCoralNoPathing(1, elevatorSubsystem, wristSubsystem));
        NamedCommands.registerCommand("Score L2",
                AutomatedScoring.scoreCoralNoPathing(2, elevatorSubsystem, wristSubsystem));
        NamedCommands.registerCommand("Score L3",
                AutomatedScoring.scoreCoralNoPathing(3, elevatorSubsystem, wristSubsystem));

                NamedCommands.registerCommand("AlignToTag2",
                new AutoPositionToTagCommand(limelightSubsystem, m_drive, 2) // ID'si 2 olan AprilTag
            );
            
            NamedCommands.registerCommand("AlignToTag3",
                new AutoPositionToTagCommand(limelightSubsystem, m_drive, 3) // ID'si 3 olan AprilTag
            );
            
            NamedCommands.registerCommand("AlignToTag4",
                new AutoPositionToTagCommand(limelightSubsystem, m_drive, 4) // ID'si 4 olan AprilTag
            );
            
            NamedCommands.registerCommand("AlignToTag5",
                new AutoPositionToTagCommand(limelightSubsystem, m_drive, 5) // ID'si 5 olan AprilTag
            );
        /*
         * NamedCommands.registerCommand("AlignToReef1",
         * new AutoPositionToTagCommand(limelightSubsystem, driveSubsystem, 4)); //
         * Speaker AprilTag ID
         * 
         * NamedCommands.registerCommand("AlignToReef2",
         * new AutoPositionToTagCommand(limelightSubsystem, driveSubsystem, 5)); // Amp
         * AprilTag ID
         * 
         * NamedCommands.registerCommand("AlignToReef3",
         * new AutoPositionToTagCommand(limelightSubsystem, driveSubsystem, 11)); //
         * Stage AprilTag ID
         */
    }

    private void configureButtonBindings() {

        new JoystickButton(driveJoystick, 9).onChange(m_drive.xCommand()); // Needs to be while true so the command ends
        new JoystickButton(driveJoystick, 2).whileTrue(m_drive.gyroReset());

        // L1, RIGHT POV BUTTON
        new POVButton(operatorJoystick, 180)
                .whileTrue(AutomatedScoring.scoreCoralNoPathing(1, elevatorSubsystem, wristSubsystem));

        // L2, RIGHT POV BUTTON
        new POVButton(operatorJoystick, 90)
                .whileTrue(AutomatedScoring.scoreCoralNoPathing(3, elevatorSubsystem, wristSubsystem));

        // L3, RIGHT POV BUTTON
        new POVButton(operatorJoystick, 0)
                .whileTrue(AutomatedScoring.scoreCoralNoPathing(2, elevatorSubsystem, wristSubsystem));

        new JoystickButton(operatorJoystick, 5).whileTrue(
                new AutoPositionToTagCommand(limelightSubsystem, m_drive, -1) // Herhangi bir AprilTag
        );
        new JoystickButton(operatorJoystick, 4) // PS5'te üçgen butonu 4 numaralı buton
                .onTrue(new InstantCommand(() -> {
                    SmartDashboard.putBoolean("Triangle Button Pressed", true);
                    shooterSubsystem.shooterButton();
                }));

        new JoystickButton(operatorJoystick, 1).whileTrue(
                new LimelightDriveCommand(m_drive, operatorJoystick, limelightSubsystem));
        new JoystickButton(operatorJoystick, 1)
                .whileTrue(new RunCommand(() -> climbSubsystem.moveAtSpeed(1.0), climbSubsystem))
                .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb(), climbSubsystem));

        new JoystickButton(operatorJoystick, 2)
                .whileTrue(new RunCommand(() -> climbSubsystem.moveAtSpeed(-1.0), climbSubsystem))
                .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb(), climbSubsystem));

        /*
         * A button for automatic positioning to all tags
         * new JoystickButton(operatorJoystick, 4)
         * .whileTrue(new AutoPositionToTagCommand(limelightSubsystem, driveSubsystem,
         * 4)); // Replace 4 with desired tag ID
         * 
         * B button for positioning to specific tag 3
         * new JoystickButton(driveJoystick, 2) // Button number 2, can be changed if
         * needed
         * .onTrue(new InstantCommand(() -> limelightSubsystem.autoPositionToTag(3)));
         * 
         * X button for positioning to specific tag 4
         * new JoystickButton(driveJoystick, 3) // Button number 3, can be changed if
         * needed
         * .onTrue(new InstantCommand(() -> limelightSubsystem.autoPositionToTag(4)));
         */
    }

    public Command getAutonomousCommand() {
        if (m_autoPositionChooser.getSelected() != null) {
            return m_autoPositionChooser.getSelected();
        } else {
            return m_drive.gyroReset();
        }
    }

    public Command getTestingCommand() {
        return new RobotSystemsCheckCommand(m_drive);
    }

    public Field2d getField() {
        return field;
    }

    public static final class UserPolicy {
        public static boolean xLocked = false;
        public static boolean isManualControlled = true;
    }
}
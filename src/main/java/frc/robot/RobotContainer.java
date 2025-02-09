// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.automation.AutomationSelector;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.automation.AutomatedScoring;
import edu.wpi.first.wpilibj.PS5Controller;

public class RobotContainer {

    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    public final VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public final WristSubsystem wristSubsystem = new WristSubsystem();
    
    public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
   
    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(driveSubsystem);

    private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
    private final PS5Controller operatorJoystick = new PS5Controller(RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);
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
                AutomatedScoring.scoreNoPathing(1, elevatorSubsystem, wristSubsystem,shooterSubsystem));
        NamedCommands.registerCommand("Score L2",
                AutomatedScoring.scoreNoPathing(2, elevatorSubsystem, wristSubsystem, shooterSubsystem));
        NamedCommands.registerCommand("Score L3",
                AutomatedScoring.scoreNoPathing(3, elevatorSubsystem, wristSubsystem, shooterSubsystem));
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
        

        new JoystickButton(operatorJoystick, PS5Controller.Button.kR1.value).onTrue(elevatorSubsystem.goToScoreSetpoint(1));

        // Buton 2: L2 seviyesine git
        new JoystickButton(operatorJoystick, PS5Controller.Button.kR2.value).onTrue(elevatorSubsystem.goToScoreSetpoint(2));

        // Buton 3: L3 seviyesine git
        new JoystickButton(operatorJoystick, PS5Controller.Button.kL1.value).onTrue(elevatorSubsystem.goToScoreSetpoint(3));


        new JoystickButton(operatorJoystick, PS5Controller.Button.kTriangle.value).whileTrue(new RunCommand(() -> shooterSubsystem.moveAtSpeed(1.0), shooterSubsystem))
        .onFalse(new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem));
       
        new JoystickButton(operatorJoystick, PS5Controller.Button.kSquare.value).whileTrue(new RunCommand(() -> climbSubsystem.moveAtSpeed(1.0), climbSubsystem))
        .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb(), climbSubsystem));

        new JoystickButton(operatorJoystick, PS5Controller.Button.kCross.value)  .whileTrue(new RunCommand(() -> climbSubsystem.moveAtSpeed(-1.0), climbSubsystem))
        .onFalse(new InstantCommand(() -> climbSubsystem.stopClimb(), climbSubsystem));
       
        // A butonu ile tüm tag'lere otomatik konumlanma
        new JoystickButton(operatorJoystick, PS5Controller.Button.kTouchpad.value) // 1 numaralı buton, gerekirse değiştirebilirsiniz
        .onTrue(new InstantCommand(() -> limelightSubsystem.autoPositionToAllTags()));
         // Tüm AprilTag'lere otomatik konumlandırma komutunu tetikler
         // B butonu ile spesifik tag 3'e konumlanma
        new JoystickButton(driveJoystick, 2) // 2 numaralı buton, gerekirse değiştirebilirsiniz
        .onTrue(new InstantCommand(() -> limelightSubsystem.autoPositionToTag(3)));
  // Spesifik olarak tag 3'e konumlandırma komutunu tetikler
        // X butonu ile spesifik tag 4'e konumlanma
        new JoystickButton(driveJoystick, 3) // 3 numaralı buton, gerekirse değiştirebilirsiniz
        .onTrue(new InstantCommand(() -> limelightSubsystem.autoPositionToTag(4)));
 // Spesifik olarak tag 4'e konumlandırma komutunu tetikler
    }

    public Command getAutonomousCommand() {
        if (m_autoPositionChooser.getSelected() != null) {
            return m_autoPositionChooser.getSelected();
        } else {
            return new InstantCommand(() -> limelightSubsystem.autoPositionToAllTags());
   // Eğer seçili otonom komut yoksa, tüm AprilTag'lere otomatik konumlandırma yapar
   
            
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
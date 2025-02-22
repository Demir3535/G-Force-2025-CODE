package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants.DrivetrainConstants;

public class LimelightDriveCommand extends Command {
    private final DriveSubsystem drive;
    private final Joystick joystick;
    private final LimelightSubsystem limelight;
    
    public LimelightDriveCommand(DriveSubsystem drive, Joystick joystick, LimelightSubsystem limelight) {
        this.drive = drive;
        this.joystick = joystick;
        this.limelight = limelight;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // Joystick değerlerini al ve yavaşlat (0.7 ile çarp)
        double xSpeed = -joystick.getRawAxis(3) * 0.7;
        double ySpeed = -joystick.getRawAxis(2) * 0.7;
        
        // Limelight dönüş değerini al
        double rot = limelight.getSteer();
        
        // Sürüşü uygula
        drive.drive(
            ySpeed, 
            xSpeed, 
            rot,
            DrivetrainConstants.FIELD_RELATIVE, 
            true
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, DrivetrainConstants.FIELD_RELATIVE, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
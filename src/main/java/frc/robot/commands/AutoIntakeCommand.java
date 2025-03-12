package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoIntakeCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final double intakeSpeed;
    private final double maxIntakeTime;
    private long startTime;
    private boolean hasDetectedCoral = false;

    /**
     * Otomatik intake komutu
     * Coral algılanırsa motor durur
     * Maksimum süre dolduğunda da durur
     * 
     * @param shooterSubsystem Shooter subsystem
     * @param intakeSpeed Intake hızı (-1.0 ile 1.0 arası)
     * @param maxIntakeTime Maksimum intake süresi (saniye)
     */
    public AutoIntakeCommand(ShooterSubsystem shooterSubsystem, double intakeSpeed, double maxIntakeTime) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSpeed = intakeSpeed;
        this.maxIntakeTime = maxIntakeTime * 1000; // Milisaniyeye çevir
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // Başlangıç durumlarını ayarla
        hasDetectedCoral = false;
        shooterSubsystem.moveAtSpeed(intakeSpeed);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Limit switch'i kontrol et - coral algılandıysa
        if (!hasDetectedCoral && shooterSubsystem.hasGameElement()) {
            // Coral algılandı, motoru durdur
            shooterSubsystem.stopShooter();
            hasDetectedCoral = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Komut bittiğinde veya kesildiğinde motoru durdur
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        // Coral algılandıysa veya maksimum süre geçtiyse komutu bitir
        return hasDetectedCoral || System.currentTimeMillis() - startTime >= maxIntakeTime;
    }
}
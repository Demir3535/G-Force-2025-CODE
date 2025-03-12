package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final double shootSpeed;
    private final double timeToShoot;
    private long startTime;

    /**
     * Limit switch denetimi olmadan otomatik atış yapan komut
     * 
     * @param shooterSubsystem Shooter subsystem
     * @param shootSpeed Atış hızı (-1.0 ile 1.0 arası)
     * @param timeToShoot Atış süresi (saniye)
     */
    public AutoShootCommand(ShooterSubsystem shooterSubsystem, double shootSpeed, double timeToShoot) {
        this.shooterSubsystem = shooterSubsystem;
        this.shootSpeed = shootSpeed;
        this.timeToShoot = timeToShoot * 1000; // Milisaniyeye çevir
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        // Motorları belirlenen hızda başlat
        shooterSubsystem.moveAtSpeed(shootSpeed);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        // Komut çalışırken sürekli yapılacak işlemler
        // Bu durumda bir şey yapmamıza gerek yok, motor zaten çalışıyor
    }

    @Override
    public void end(boolean interrupted) {
        // Komut bittiğinde veya kesildiğinde motoru durdur
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        // Belirlenen süre geçtiyse komutu bitir
        return System.currentTimeMillis() - startTime >= timeToShoot;
    }
}
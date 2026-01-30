package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Test modunda Flywheel PID ayarlaması için komut.
 * 
 * <p>
 * Dashboard'dan hedef RPM ve PID değerlerini okuyarak
 * canlı tuning yapılmasını sağlar.
 * </p>
 * 
 * <h2>Kullanım:</h2>
 * <ol>
 * <li>DriverStation'dan Test moduna geçin</li>
 * <li>Elastic/Shuffleboard'dan "Shooter/Test/Target RPM" değerini
 * ayarlayın</li>
 * <li>PID değerlerini ayarlayarak yanıtı gözlemleyin</li>
 * </ol>
 */
public class FlywheelTestCommand extends Command {

    private final ShooterSubsystem shooterSubsystem;

    // Test mode için hedef RPM (Dashboard'dan ayarlanabilir)
    private final TunableNumber testTargetRPM = new TunableNumber("Shooter/Test", "Target RPM", 3000.0);

    // Logging için son değerler
    private double lastTargetRPM = 0;

    public FlywheelTestCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("[FlywheelTest] Test modu başlatıldı - PID tuning aktif");
    }

    @Override
    public void execute() {
        // Dashboard'dan hedef RPM'i oku
        double targetRPM = testTargetRPM.get();

        // Hedef değiştiğinde logla
        if (targetRPM != lastTargetRPM) {
            System.out.println("[FlywheelTest] Hedef RPM değişti: " + targetRPM);
            lastTargetRPM = targetRPM;
        }

        // Flywheel'e hedefi uygula
        shooterSubsystem.setFlywheelRPM(targetRPM);

        // AdvantageKit ile logla (Elastic'te görüntülenebilir)
        Logger.recordOutput("Shooter/Test/TargetRPM", targetRPM);
        Logger.recordOutput("Shooter/Test/ActualRPM", shooterSubsystem.getFlywheelActualRPM());
        Logger.recordOutput("Shooter/Test/Error", targetRPM - shooterSubsystem.getFlywheelActualRPM());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        System.out.println("[FlywheelTest] Test modu sonlandırıldı");
    }

    @Override
    public boolean isFinished() {
        return false; // Test modunda sürekli çalışır
    }
}

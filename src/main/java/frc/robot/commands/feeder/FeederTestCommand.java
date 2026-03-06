package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Test modunda Feeder PID ayarlaması için komut.
 * Feeder RPM değeri dashboard'dan girilerek canlı tuning yapılır.
 * 
 * <h2>Dashboard Erişimi:</h2>
 * <ul>
 * <li><code>/Tuning/Feeder/Test/Target RPM</code> - Feeder hedef hızı
 * (RPM)</li>
 * </ul>
 * 
 * <h2>Mevcut PID Tuning Yolları (FeederSubsystem):</h2>
 * <ul>
 * <li><code>/Tuning/Feeder/kP</code></li>
 * <li><code>/Tuning/Feeder/kI</code></li>
 * <li><code>/Tuning/Feeder/kD</code></li>
 * <li><code>/Tuning/Feeder/kFF</code></li>
 * </ul>
 */
public class FeederTestCommand extends Command {

    private final FeederSubsystem feeder;

    // Dashboard'dan girilen hedef değer
    private final TunableNumber testTargetRPM;

    public FeederTestCommand(FeederSubsystem feeder) {
        this.feeder = feeder;
        addRequirements(feeder);

        // TunableNumber - Dashboard'dan RPM girilecek
        testTargetRPM = new TunableNumber("Feeder/Test", "Target RPM", 0.0);
    }

    @Override
    public void initialize() {
        // Manual override aç - Shooter'dan bağımsız çalışsın
        feeder.enableManualOverride();

        System.out.println("========================================");
        System.out.println("[FeederTest] Test modu başlatıldı!");
        System.out.println("[FeederTest] Dashboard: /Tuning/Feeder/Test/");
        System.out.println("[FeederTest] Target RPM: /Tuning/Feeder/Test/Target RPM");
        System.out.println("[FeederTest] PID tuning: /Tuning/Feeder/");
        System.out.println("[FeederTest] Manual Override ENABLED - Shooter bağımsız");
        System.out.println("========================================");
    }

    @Override
    public void execute() {
        // =========== FEEDER ===========
        double targetRPM = testTargetRPM.get();

        if (Math.abs(targetRPM) > 10) {
            feeder.setManualOverrideRPM(targetRPM);
        } else {
            feeder.manualStop();
        }

        // =========== LOG ===========
        Logger.recordOutput("Tuning/Feeder/Test/TargetRPM", targetRPM);
        Logger.recordOutput("Tuning/Feeder/Test/ActualRPM", feeder.getVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/Test/AtTarget", feeder.isAtTarget());
    }

    @Override
    public void end(boolean interrupted) {
        feeder.manualStop();
        feeder.disableManualOverride();
        System.out.println("[FeederTest] Test modu sonlandırıldı - Manual Override kapatıldı");
    }

    @Override
    public boolean isFinished() {
        return false; // Test modunda sürekli çalışır
    }
}


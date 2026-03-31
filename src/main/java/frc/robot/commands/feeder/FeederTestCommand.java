package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Test modunda Feeder PID ayarlaması için komut.
 * Indexer ve Kicker RPM değerleri dashboard'dan girilerek canlı tuning yapılır.
 * 
 * <h2>Dashboard Erişimi:</h2>
 * <ul>
 * <li><code>/Tuning/Feeder/Test/Indexer Target RPM</code></li>
 * <li><code>/Tuning/Feeder/Test/Kicker Target RPM</code></li>
 * </ul>
 * 
 * <h2>Mevcut PID Tuning Yolları (FeederSubsystem):</h2>
 * <ul>
 * <li><code>/Tuning/Feeder/Indexer/kP, kI, kD, kFF</code></li>
 * <li><code>/Tuning/Feeder/Kicker/kP, kI, kD, kFF</code></li>
 * </ul>
 */
public class FeederTestCommand extends Command {

    private final FeederSubsystem feeder;

    private final TunableNumber testIndexerRPM;
    private final TunableNumber testKickerRPM;

    public FeederTestCommand(FeederSubsystem feeder) {
        this.feeder = feeder;
        addRequirements(feeder);

        testIndexerRPM = new TunableNumber("Tuning/Feeder/Test", "Indexer Target RPM", 0.0);
        testKickerRPM = new TunableNumber("Tuning/Feeder/Test", "Kicker Target RPM", 0.0);
    }

    @Override
    public void initialize() {
        feeder.enableManualOverride();

        System.out.println("========================================");
        System.out.println("[FeederTest] Test modu başlatıldı!");
        System.out.println("[FeederTest] Dashboard: /Tuning/Feeder/Test/");
        System.out.println("[FeederTest] Indexer RPM: /Tuning/Feeder/Test/Indexer Target RPM");
        System.out.println("[FeederTest] Kicker RPM:  /Tuning/Feeder/Test/Kicker Target RPM");
        System.out.println("[FeederTest] PID tuning:  /Tuning/Feeder/Indexer/ & /Tuning/Feeder/Kicker/");
        System.out.println("[FeederTest] Manual Override ENABLED");
        System.out.println("========================================");
    }

    @Override
    public void execute() {
        double indexerRPM = testIndexerRPM.get();
        double kickerRPM = testKickerRPM.get();

        // Indexer
        if (Math.abs(indexerRPM) > 10) {
            feeder.setManualIndexerRPM(indexerRPM);
        } else {
            feeder.setManualIndexerRPM(0);
        }

        // Kicker
        if (Math.abs(kickerRPM) > 10) {
            feeder.setManualKickerRPM(kickerRPM);
        } else {
            feeder.setManualKickerRPM(0);
        }

        // Telemetri
        Logger.recordOutput("Tuning/Feeder/Test/IndexerTargetRPM", indexerRPM);
        Logger.recordOutput("Tuning/Feeder/Test/IndexerActualRPM", feeder.getIndexerVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/Test/KickerTargetRPM", kickerRPM);
        Logger.recordOutput("Tuning/Feeder/Test/KickerActualRPM", feeder.getKickerVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/Test/AtTarget", feeder.isAtTarget());
    }

    @Override
    public void end(boolean interrupted) {
        feeder.manualStop();
        feeder.disableManualOverride();
        System.out.println("[FeederTest] Test modu sonlandırıldı — Manual Override kapatıldı");
    }

    @Override
    public boolean isFinished() {
        return false; // Test modunda sürekli çalışır
    }
}

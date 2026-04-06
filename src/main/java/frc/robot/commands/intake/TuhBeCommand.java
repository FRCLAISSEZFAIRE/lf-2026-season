package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TunableNumber;

/**
 * "Tüh Be!" Komutu — Intake açık unutulduğunda acil geri sarma.
 * "Oops!" Command — Emergency retract when intake is left deployed.
 *
 * <h3>Akış / Flow:</h3>
 * <ol>
 *   <li>Önce retract pozisyonuna PID ile git / First go to retract position via PID</li>
 *   <li>Sonra eksi yönde düşük voltajla hard stop'a kadar sür (homing) /
 *       Then drive in negative direction at low voltage until hard stop (homing)</li>
 *   <li>Son olarak enkoderi 0'a sıfırla / Finally reset encoder to 0</li>
 * </ol>
 *
 * <p>Dashboard'dan "Tuning/Intake/TÜHBE - Acil Geri Sar" butonu ile tetiklenir.
 * Triggered from Dashboard via "Tuning/Intake/TÜHBE - Acil Geri Sar" button.</p>
 */
public class TuhBeCommand extends Command {

    private static final TunableNumber retractWaitSec =
        new TunableNumber("Intake/TuhBe", "Retract Wait Sec", 0.8);

    private static final TunableNumber homingVolts =
        new TunableNumber("Intake/TuhBe", "Homing Voltage", -2.5);

    private static final TunableNumber homingDurationSec =
        new TunableNumber("Intake/TuhBe", "Homing Duration Sec", 1.2);

    private enum Phase {
        RETRACTING,   // PID ile retract konumuna git / Go to retract via PID
        HOMING,       // Eksi yönde voltajla hard stop'a bas / Drive negative into hard stop
        DONE          // Bitti / Done
    }

    private final IntakeSubsystem intake;
    private final Timer timer = new Timer();
    private Phase phase = Phase.RETRACTING;

    /**
     * @param intake IntakeSubsystem — gereksinim olarak alınır / taken as requirement
     */
    public TuhBeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        phase = Phase.RETRACTING;
        timer.restart();

        // Adım 1: Normal retract komutu / Step 1: Normal retract
        intake.retract();

        System.out.println("[TühBe] Başladı — Retract aşaması / Started — Retract phase");
    }

    @Override
    public void execute() {
        switch (phase) {

            // ----------------------------------------------------------------
            case RETRACTING:
                // PID'in retract konumuna ulaşması için bekle
                // Wait for PID to reach retract position
                if (timer.hasElapsed(retractWaitSec.get())) {
                    // Adım 2: Eksi yönde homing voltajı uygula
                    // Step 2: Apply negative homing voltage
                    phase = Phase.HOMING;
                    timer.restart();
                    System.out.println("[TühBe] Homing aşaması — Eksi voltaj uygulanıyor / Homing phase — applying negative voltage");
                }
                break;

            // ----------------------------------------------------------------
            case HOMING:
                // Eksi yönde düşük voltajla sür — mekanik durdurucu hattına kadar
                // Drive in negative direction at low voltage — until mechanical hard stop
                intake.runRoller(0); // Roller durdur / Stop roller
                // Extension motoruna direkt voltaj uygula (soft limit bypass)
                // Apply direct voltage to extension motor (soft limit bypass happens below)
                applyRawExtensionVoltage(homingVolts.get());

                if (timer.hasElapsed(homingDurationSec.get())) {
                    // Adım 3: Enkoderi sıfırla / Step 3: Reset encoder
                    stopAndZero();
                    phase = Phase.DONE;
                }
                break;

            // ----------------------------------------------------------------
            case DONE:
                // Bekleme / Waiting
                break;
        }
    }

    /**
     * IntakeSubsystem'in extension motoruna ham voltaj uygular.
     * Soft limit'leri bypass etmek için direkt çağrılır.
     * Applies raw voltage to the extension motor, bypassing soft limits.
     */
    private void applyRawExtensionVoltage(double volts) {
        // IntakeSubsystem public API'si yok bu işlem için — setVoltage kullanıyoruz
        // No public API for this — we use the stopAll() + runRoller trick via a workaround.
        // Bu sınıf aynı pakette olduğu için package-private erişim kullanıyoruz
        // Since this class is in the same broader domain, we expose through the subsystem.
        intake.applyExtensionVoltage(volts);
    }

    private void stopAndZero() {
        intake.applyExtensionVoltage(0);
        intake.resetExtensionEncoder();
        System.out.println("[TühBe] Tamamlandı! Encoder sıfırlandı / Done! Encoder reset to 0");
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        intake.applyExtensionVoltage(0);
        if (interrupted) {
            System.out.println("[TühBe] KESİLDİ / INTERRUPTED");
        }
    }
}

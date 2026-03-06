package frc.robot.commands.intake;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Zamanlı otomatik intake komutu.
 * Robot-relative ileri hareket ederken intake roller ve feeder çalıştırır.
 * Süre dolduğunda durur.
 * 
 * <p>
 * Sürüş hızı TunableNumber ile Dashboard'dan ayarlanabilir,
 * değer RIO'ya kaydedilir.
 * </p>
 */
public class AutoIntakeCommand extends Command {
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final FeederSubsystem feeder;
    private final double durationSeconds;

    private final Timer timer = new Timer();

    // Intake sırasında ileri sürüş hızı (m/s) — Dashboard'dan ayarlanabilir
    private static final TunableNumber intakeDriveSpeed = new TunableNumber(
            "Auto", "IntakeDriveSpeed", 0.3); // Varsayılan: 0.3 m/s (çok yavaş)

    /**
     * Zamanlı intake komutu oluşturur.
     * Robot ileri hareket ederken intake roller ve feeder çalıştırır.
     *
     * @param drive           Drive alt sistemi (ileri sürüş için)
     * @param intake          Intake alt sistemi
     * @param feeder          Feeder alt sistemi
     * @param durationSeconds Çalışma süresi (saniye)
     */
    public AutoIntakeCommand(DriveSubsystem drive, IntakeSubsystem intake,
            FeederSubsystem feeder, double durationSeconds) {
        this.drive = drive;
        this.intake = intake;
        this.feeder = feeder;
        this.durationSeconds = durationSeconds;

        addRequirements(drive, intake, feeder);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("[AutoIntake] Başlatıldı — Süre: " + durationSeconds
                + "s, Hız: " + intakeDriveSpeed.get() + " m/s");
    }

    @Override
    public void execute() {
        // Intake ve Feeder çalıştır
        intake.runRollerRPM(4800); // 4800 RPM
        feeder.feed();

        // Robot-relative ileri sürüş (intake yönüne doğru)
        double speed = intakeDriveSpeed.get();
        drive.runVelocity(new ChassisSpeeds(speed, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        intake.stopRoller();
        feeder.stop();
        drive.stop();
        System.out.println("[AutoIntake] Durduruldu" + (interrupted ? " (kesildi)" : ""));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}

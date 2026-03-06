package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;

import java.util.function.Supplier;

/**
 * Pose Tabanlı Atış Komutu.
 * 
 * - Robot pozisyonuna göre sürekli hedefleme günceller (Taret, Hood, Flywheel).
 * - Feeder SADECE shooter hazır olduğunda çalışır (PID kontrolü geçerse).
 * - "Ateşi Tut" mantığı: Flywheel hazır olduğunda feeder kilitlenerek çalışır.
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final Supplier<Pose2d> poseSupplier;

    // Ateş kilidi — bir kez başladıktan sonra feeder durmaz
    private boolean hasShot = false;

    /**
     * Yeni bir ShootCommand oluşturur.
     * 
     * @param shooter      Shooter alt sistemi
     * @param feeder       Feeder alt sistemi
     * @param poseSupplier Robot pose kaynağı (DriveSubsystem'den)
     */
    public ShootCommand(
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            Supplier<Pose2d> poseSupplier) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.poseSupplier = poseSupplier;

        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        hasShot = false;
        System.out.println("[ShootCommand] Başlatıldı — Pose Hedefleme");
    }

    @Override
    public void execute() {
        // 1. Güncel robot pozisyonunu al
        Pose2d currentPose = poseSupplier.get();
        if (currentPose == null) {
            feeder.stop();
            return;
        }

        // 2. Hedefleme güncelle (Taret, Hood, RPM)
        // İttifak alanındaysa Hub'a at, değilse Pass yap
        if (shooter.isInAllianceZone()) {
            shooter.updateAiming(currentPose);
        } else {
            shooter.updateAimingForPass(currentPose);
        }

        // 3. "Ateşi Tut" Mantığı — LATCH (Kilitleme)
        // Bir kez ateşlemeye başladıktan sonra RPM düşmesi göz ardı edilir.
        // Sadece komut sonlandığında (tetik bırakıldığında) durur.

        // ESNEK HAZIR KONTROLÜ:
        // Sadece Flywheel RPM hazır olmasını kontrol eder.
        // Taret/Hood hizalaması kontrol edilmez (robot hareket ederken takılmayı
        // önler).
        boolean ready = shooter.isFlywheelAtTarget();

        if (ready && !hasShot) {
            hasShot = true; // Ateşleme başladı olarak işaretle
        }

        // KİLİTLEME MANTIĞI:
        // Hazırsa VEYA daha önce ateş başladıysa (kilitli) → FEEDER ÇALIŞTIR
        if (ready || hasShot) {
            feeder.feed();
        } else {
            // Hazır değil ve henüz başlamadı → Bekle
            feeder.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Tetik bırakıldığında her şeyi durdur
        shooter.stopFlywheel();
        feeder.stop();
        System.out.println("[ShootCommand] Durduruldu");
    }

    @Override
    public boolean isFinished() {
        // Buton basılı tutulduğu sürece çalışır
        return false;
    }
}

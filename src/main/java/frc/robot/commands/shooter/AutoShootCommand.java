package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.TunableNumber;

import java.util.function.Supplier;

/**
 * Otonom ve Dashboard için kullanılabilecek otomatik atış komutu.
 *
 * <p>Pose bazlı nişan alır (hub veya pass), flywheel'i çalıştırır ve
 * ayarlanan süre boyunca feeder'ı çalıştırarak atış yapar.</p>
 *
 * <h3>Dashboard Ayarları:</h3>
 * <ul>
 *   <li>{@code Tuning/Shooter/Test/AutoShoot Duration Sec} — atış süresi (sn)</li>
 * </ul>
 *
 * <h3>AdvantageScope Logları:</h3>
 * <ul>
 *   <li>{@code Tuning/Shooter/Aiming/TargetTurretAngle} — hesaplanan taret açısı (°)</li>
 *   <li>{@code Tuning/Shooter/Turret/ActualDeg}         — gerçek taret açısı     (°)</li>
 *   <li>{@code Tuning/Shooter/Aiming/Distance}          — hedefe mesafe           (m)</li>
 * </ul>
 */
public class AutoShootCommand extends Command {

    /** Dashboard'dan ayarlanabilir atış süresi (saniye). */
    public static final TunableNumber shootDurationSec =
            new TunableNumber("Shooter/Test", "AutoShoot Duration Sec", 2.0);

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final Supplier<Pose2d> poseSupplier;

    private final Timer timer = new Timer();

    /**
     * @param shooter      ShooterSubsystem
     * @param feeder       FeederSubsystem
     * @param poseSupplier Robot konum kaynağı (drive::getPose)
     */
    public AutoShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder, Supplier<Pose2d> poseSupplier) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.poseSupplier = poseSupplier;
        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        System.out.println("[AutoShoot] Başlatıldı — süre=" + shootDurationSec.get() + "s");
    }

    @Override
    public void execute() {
        Pose2d currentPose = poseSupplier.get();
        if (currentPose == null) {
            feeder.stop();
            return;
        }

        // Pose bazlı nişan: taret açısı, hood ve flywheel otomatik hesaplanır.
        // Hesaplanan açılar AdvantageScope'ta "Tuning/Shooter/Aiming/*" altında görünür.
          if (shooter.isInAllianceZone()) {
            shooter.updateAiming(currentPose);
            } else {
                shooter.updateAimingForPass(currentPose);
            }
        // Feeder her daim çalışır (flywheel hazır olmayı bekleme —
        // bu test komutudur, anında ateşleme yapıyoruz)
        feeder.feed();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(shootDurationSec.get());
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        shooter.stopFlywheel();
        System.out.println("[AutoShoot] Bitti — " + (interrupted ? "iptal edildi" : "süre doldu"));
    }
}

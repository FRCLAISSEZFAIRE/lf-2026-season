package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.constants.IntakeConstants;
import frc.robot.util.TunableNumber;

import java.util.function.Supplier;

/**
 * Otonom için zamanlı atış komutu — ShootCommand'ın aynısı,
 * fakat belirtilen süre dolunca otomatik olarak durur.
 *
 * <p>
 * Pose bazlı nişan alır, "Ateşi Tut" kilitleme mantığı uygular,
 * intake ajitasyonu yapar ve süre dolunca tüm alt sistemleri durdurur.
 * </p>
 *
 * <h3>Dashboard Ayarları:</h3>
 * <ul>
 * <li>{@code Tuning/Shooter/Test/AutoShoot Duration Sec} — atış süresi
 * (sn)</li>
 * </ul>
 */
public class AutoShootCommand extends Command {

    /** Dashboard'dan ayarlanabilir atış süresi (saniye). */
    public static final TunableNumber shootDurationSec = new TunableNumber("Shooter/Test", "AutoShoot Duration Sec",
            5.0);

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final Supplier<Pose2d> poseSupplier;

    // Ateş kilidi — bir kez başladıktan sonra feeder durmaz
    private boolean hasShot = false;

    // Intake deploy/retract döngüsü
    private final Timer cycleTimer = new Timer();
    private boolean intakeDeployed = false;

    // Zamanlayıcı
    private final Timer timer = new Timer();

    // Tunable: döngü süresi (saniye) ve düşük RPM (ShootCommand ile paylaşılır)
    private static final TunableNumber intakeCycleTime = new TunableNumber("Shooter", "IntakeCycleTimeSec", 1.5);
    private static final TunableNumber intakeShootRPM = new TunableNumber("Shooter", "IntakeShootRPM", 1500.0);

    /**
     * @param shooter      ShooterSubsystem
     * @param feeder       FeederSubsystem
     * @param drive        DriveSubsystem
     * @param intake       IntakeSubsystem
     * @param poseSupplier Robot konum kaynağı (drive::getPose)
     */
    public AutoShootCommand(
            ShooterSubsystem shooter,
            FeederSubsystem feeder,
            DriveSubsystem drive,
            IntakeSubsystem intake,
            Supplier<Pose2d> poseSupplier) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.drive = drive;
        this.intake = intake;
        this.poseSupplier = poseSupplier;

        addRequirements(shooter, feeder, intake);
    }

    @Override
    public void initialize() {
        hasShot = false;
        intakeDeployed = false;
        cycleTimer.restart();
        timer.restart();

        // Atış süresince dönüş merkezini taretin fiziksel merkezine ayarla
        drive.setCenterOfRotation(shooter.getTurretCenterOfRotation());

        System.out.println("[AutoShoot] Başlatıldı — süre=" + shootDurationSec.get() + "s");
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
        if (shooter.isInAllianceZone()) {
            shooter.updateAiming(currentPose);
        } else {
            shooter.updateAimingForPass(currentPose);
        }

        // 3. "Ateşi Tut" Mantığı — LATCH (Kilitleme)
        boolean ready = shooter.isFlywheelAtTarget();

        if (ready && !hasShot) {
            hasShot = true;
        }

        // KİLİTLEME MANTIĞI:
        if (ready || hasShot) {
            feeder.feed();
        } else {
            feeder.stop();
        }

        // 4. Intake Ajitasyonu — yavaş deploy/retract döngüsü + düşük RPM roller
        intake.runRollerRPM(intakeShootRPM.get());

        if (cycleTimer.hasElapsed(intakeCycleTime.get())) {
            intakeDeployed = !intakeDeployed;
            if (intakeDeployed) {
                intake.setExtensionPosition(IntakeConstants.kExtensionDeployedCm);
            } else {
                intake.setExtensionPosition(IntakeConstants.kExtensionHalfRetractedCm);
            }
            cycleTimer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(shootDurationSec.get());
    }

    @Override
    public void end(boolean interrupted) {
        // Her şeyi durdur
        shooter.stopFlywheel();
        feeder.stop();
        intake.stopRoller();
        intake.setExtensionPosition(IntakeConstants.kExtensionRetractedCm);

        // Dönüş merkezini tekrar robot merkezine sıfırla
        drive.resetCenterOfRotation();
        cycleTimer.stop();
        timer.stop();

        System.out.println("[AutoShoot] Bitti — " + (interrupted ? "iptal edildi" : "süre doldu"));
    }
}

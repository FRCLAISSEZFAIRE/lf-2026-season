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
 * Pose Tabanlı Atış Komutu.
 * 
 * - Robot pozisyonuna göre sürekli hedefleme günceller (Taret, Hood, Flywheel).
 * - Feeder SADECE shooter hazır olduğunda çalışır (PID kontrolü geçerse).
 * - "Ateşi Tut" mantığı: Flywheel hazır olduğunda feeder kilitlenerek çalışır.
 * - Intake yavaşça deploy/retract döngüsü yapar ve düşük RPM'de roller
 * çalıştırır
 * (top ajitasyonu — topları feeder'a itmek için).
 */
public class ShootCommand extends Command {

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

    // Tunable: döngü süresi (saniye) ve düşük RPM
    private static final TunableNumber intakeCycleTime = new TunableNumber("Shooter", "IntakeCycleTimeSec", 1.5);
    private static final TunableNumber intakeShootRPM = new TunableNumber("Shooter", "IntakeShootRPM", 1500.0);

    /**
     * Yeni bir ShootCommand oluşturur.
     * 
     * @param shooter      Shooter alt sistemi
     * @param feeder       Feeder alt sistemi
     * @param drive        Drive alt sistemi
     * @param intake       Intake alt sistemi
     * @param poseSupplier Robot pose kaynağı (DriveSubsystem'den)
     */
    public ShootCommand(
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

        // Atış süresince dönüş merkezini taretin fiziksel merkezine ayarla
        drive.setCenterOfRotation(shooter.getTurretCenterOfRotation());

        System.out.println("[ShootCommand] Başlatıldı — Pose Hedefleme, Dönüş Merkezi Taret, Intake Ajitasyonu Aktif");
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
    public void end(boolean interrupted) {
        // Tetik bırakıldığında her şeyi durdur
        shooter.stopFlywheel();
        feeder.stop();
        intake.stopRoller();
        intake.setExtensionPosition(IntakeConstants.kExtensionRetractedCm);

        // Dönüş merkezini tekrar robot merkezine sıfırla
        drive.resetCenterOfRotation();
        cycleTimer.stop();

        System.out.println("[ShootCommand] Durduruldu");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

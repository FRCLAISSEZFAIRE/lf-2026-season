package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

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
    private final DigitalInput mz80_8;
    private final DigitalInput mz80_9;

    // Ateş kilidi — bir kez başladıktan sonra feeder durmaz
    private boolean hasShot = false;

    // Tunable: atış sırasında roller RPM
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
            DigitalInput mz80_8,
            DigitalInput mz80_9,
            Supplier<Pose2d> poseSupplier) {
        this.shooter = shooter;
        this.feeder = feeder;
        this.drive = drive;
        this.intake = intake;
        this.poseSupplier = poseSupplier;
        this.mz80_8 = mz80_8;
        this.mz80_9 = mz80_9;

        addRequirements(shooter, feeder, intake);
    }

    @Override
    public void initialize() {
        hasShot = false;

        // Atış süresince dönüş merkezini taretin fiziksel merkezine ayarla
        drive.setCenterOfRotation(shooter.getTurretCenterOfRotation());

        // Atış modunu aktif et (hız limiti)
        drive.setShootMode(true);

        // ▶️ Kritik: initialize'da hemen hedefleme al — flywheel ilk frame'den itibaren
        // ısınsın
        // ▶️ Critical: immediately update aiming in initialize — flywheel starts
        // warming from frame 0
        Pose2d initPose = poseSupplier.get();
        if (initPose != null) {
            if (shooter.isInAllianceZone()) {
                shooter.updateAiming(initPose, drive.getFieldVelocity());
            } else {
                shooter.updateAimingForPass(initPose, drive.getFieldVelocity());
            }
        }

        System.out.println("[ShootCommand] Başlatıldı — Pose Hedefleme, Dönüş Merkezi Taret");
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
            shooter.updateAiming(currentPose, drive.getFieldVelocity());
        } else {
            // For now, let's keep pass aiming static or add overload if needed.
            shooter.updateAimingForPass(currentPose, drive.getFieldVelocity());
        }

        // 3. "Ateşi Tut" Mantığı — LATCH (Kilitleme)
        // Tum sistemler hazır olduğunda ateşle (flywheel + hood + taret)
        // Fire when ALL systems ready (flywheel + hood + turret)
        boolean ready = shooter.isReadyToShoot();

        if (ready && !hasShot) {
            hasShot = true;
        }

        // KİLİTLEME MANTIĞI:
        // Indexer: ball konumlanması için her zaman çalışır.
        // Kicker: flywheel hedef hızda olduğunda devreye girer (erken ateşlemeyi önler).
        if (ready || hasShot) {
            feeder.feedIndexer();
            if (shooter.isFlywheelAtTarget()) {
                feeder.feedKicker();
            } else {
                feeder.stopKicker();
            }
        } else {
            feeder.stop();
        }

        // 4. Intake roller — sadece düşük RPM'de roller çalıştır (extension'a dokunma)
        intake.runRollerRPM(intakeShootRPM.get());
    }

    @Override
    public void end(boolean interrupted) {
        // Tetik bırakıldığında her şeyi durdur
        shooter.stopFlywheel();
        feeder.stop();
        intake.stopRoller();
        // Extension'a dokunma — intake konum neredeyse orada kalsın
        // Atış bittiğinde hood'u tekrar mekanik sınıra (30°) yasla ve kalibre et
        // Home hood back to mechanical stop (30°) and recalibrate on end
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(new HomeHoodCommand(shooter));
        // Dönüş merkezini tekrar robot merkezine sıfırla
        drive.resetCenterOfRotation();

        // Atış modunu kapat (hız limiti kaldır)
        drive.setShootMode(false);

        System.out.println("[ShootCommand] Durduruldu");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

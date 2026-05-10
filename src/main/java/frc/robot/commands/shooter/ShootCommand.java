package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;

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

        // Atış süresince dönüş merkezini taretin fiziksel merkezine ayarla
        drive.setCenterOfRotation(shooter.getTurretCenterOfRotation());

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
            shooter.updateAiming(currentPose);
        } else {
            // For now, let's keep pass aiming static or add overload if needed.
            shooter.updateAimingForPass(currentPose);
        }

        // 3. "Ateşi Tut" Mantığı — LATCH (Kilitleme)
        boolean ready = shooter.isFlywheelAtTarget();

        if (ready && !hasShot) {
            hasShot = true;
            edu.wpi.first.math.geometry.Translation2d hubLoc = shooter.getTargetHub();
            System.out.println("[Shot Log] Atış yapıldı! Robot Konumu: X: " + String.format("%.2f", currentPose.getX()) + "m, Y: " + String.format("%.2f", currentPose.getY()) + "m, Açısı: " + String.format("%.1f", currentPose.getRotation().getDegrees()) + "° | Hedef Hub: X: " + String.format("%.2f", hubLoc.getX()) + "m, Y: " + String.format("%.2f", hubLoc.getY()) + "m | Taret Açısı: " + String.format("%.1f", shooter.getTurretAngle()) + "°");
        }

        // KİLİTLEME MANTIĞI:
        if (ready || hasShot) {
            feeder.feed();
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
        shooter.setHoodAngle(0);
        // Dönüş merkezini tekrar robot merkezine sıfırla
        drive.resetCenterOfRotation();

        System.out.println("[ShootCommand] Durduruldu");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import frc.robot.util.TunableNumber;

import java.util.function.Supplier;

/**
 * Timed shooting command for autonomous - same as ShootCommand,
 * but automatically stops after the specified duration.
 *
 * <p>
 * Uses pose-based aiming, applies fire-latch logic,
 * runs intake agitation, and stops all subsystems when time expires.
 * </p>
 *
 * <h3>Dashboard Settings:</h3>
 * <ul>
 * <li>{@code Tuning/Shooter/Test/AutoShoot Duration Sec} - shoot duration
 * (sec)</li>
 * </ul>
 */
public class AutoShootCommand extends Command {

    /** Dashboard-adjustable shoot duration (seconds). */
    public static final TunableNumber shootDurationSec = new TunableNumber("Shooter/Test", "AutoShoot Duration Sec",
            5.0);

    private final ShooterSubsystem shooter;
    private final FeederSubsystem feeder;
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final Supplier<Pose2d> poseSupplier;

    // Fire latch - once started, feeder keeps running
    private boolean hasShot = false;

    // Timer for auto-timeout
    private final Timer timer = new Timer();

    // Tunable: atış sırasında roller RPM (ShootCommand ile paylaşılır)
    private static final TunableNumber intakeShootRPM = new TunableNumber("Shooter", "IntakeShootRPM", 1500.0);

    /**
     * @param shooter      ShooterSubsystem
     * @param feeder       FeederSubsystem
     * @param drive        DriveSubsystem
     * @param intake       IntakeSubsystem
     * @param poseSupplier Robot pose supplier (drive::getPose)
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

        if (intake != null) {
            addRequirements(shooter, feeder, intake);
        } else {
            addRequirements(shooter, feeder);
        }
    }

    @Override
    public void initialize() {
        hasShot = false;
        timer.restart();

        // Atış süresince dönüş merkezini taretin fiziksel merkezine ayarla
        drive.setCenterOfRotation(shooter.getTurretCenterOfRotation());

        // Atış modunu aktif et (hız limiti)
        drive.setShootMode(true);

        // ▶️ Kritik: initialize'da hemen hedefleme al — flywheel ilk frame'den itibaren ısınsın
        // ▶️ Critical: immediately update aiming in initialize — flywheel starts warming from frame 0
        Pose2d initPose = poseSupplier.get();
        if (initPose != null) {
            if (shooter.isInAllianceZone()) {
                shooter.updateAiming(initPose, drive.getFieldVelocity());
            } else {
                shooter.updateAimingForPass(initPose, drive.getFieldVelocity());
            }
        }

        System.out.println("[AutoShoot] Başlatıldı — Süre: " + shootDurationSec.get() + "s");
    }

    @Override
    public void execute() {
        // 1. Get current robot pose
        Pose2d currentPose = poseSupplier.get();
        if (currentPose == null) {
            feeder.stop();
            return;
        }

        // 2. Update aiming (Turret, Hood, RPM)
        if (shooter.isInAllianceZone()) {
            shooter.updateAiming(currentPose, drive.getFieldVelocity());
        } else {
            shooter.updateAimingForPass(currentPose, drive.getFieldVelocity());
        }

        // 3. "Ateşi Tut" Mantığı — LATCH (Kilitleme)
        // Tüm sistemler hazır olduğunda ateşle (flywheel + hood + taret)
        boolean ready = shooter.isReadyToShoot();

        if (ready && !hasShot) {
            hasShot = true;
        }

        // LATCH LOGIC:
        if (ready || hasShot) {
            feeder.feed();
        } else {
            feeder.stop();
        }

        // 4. Intake roller — sadece roller çalıştır, extension'a dokunma
        if (intake != null) {
            intake.runRollerRPM(intakeShootRPM.get());
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(shootDurationSec.get());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop everything
        shooter.stopFlywheel();
        feeder.stop();
        if (intake != null) {
            intake.stopRoller();
            // Extension'a dokunma — intake konum neredeyse orada kalsın
        }

        // Reset center of rotation back to robot center
        drive.resetCenterOfRotation();

        // Atış bittiğinde hood'u tekrar mekanik sınıra (30°) yasla ve kalibre et
        // Home hood back to mechanical stop (30°) and recalibrate on end
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(new HomeHoodCommand(shooter));

        // Atış modunu kapat (hız limiti kaldır)
        drive.setShootMode(false);

        timer.stop();

        System.out.println("[AutoShoot] Finished - " + (interrupted ? "interrupted" : "timed out"));
    }
}

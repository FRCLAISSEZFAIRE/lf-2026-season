package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Shooter için varsayılan (default) standby komutu.
 *
 * <p>Robot hub'a {@code PRE_AIM_DISTANCE_METERS} içine girdiğinde:
 * <ul>
 *   <li>Hood ve turret, mevcut mesafeye göre atış pozisyonuna smooth şekilde getirilir.</li>
 *   <li>Flywheel, beklenen RPM'e pre-spin yapar → shoot butonuna basılınca
 *       {@code isReadyToShoot()} neredeyse anında {@code true} döner.</li>
 *   <li>Lead angle kompanzasyonu ({@code fieldVelocity}) hareket halinde de çalışır.</li>
 * </ul>
 *
 * <p>Uzaktayken flywheel durdurulur, hood ve turret dinlenme pozisyonuna alınır.
 *
 * <p>ShootCommand (veya diğer shooter komutları) başladığında bu komut
 * interrupt edilir ve bittikten sonra otomatik olarak tekrar devreye girer.
 */
public class ShooterStandbyCommand extends Command {

    private final ShooterSubsystem shooter;
    private final DriveSubsystem drive;

    public ShooterStandbyCommand(ShooterSubsystem shooter, DriveSubsystem drive) {
        this.shooter = shooter;
        this.drive = drive;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Başlangıçta yeniden kontrol edilecek — execute() halledecek.
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        ChassisSpeeds speeds = drive.getFieldVelocity();

        double distance = shooter.getDistanceToHub();

        if (shooter.isInAllianceZone() && distance < ShooterConstants.PRE_AIM_DISTANCE_METERS.get()) {
            // Menzil içinde: hood + turret hedefleme + flywheel pre-spin
            // fieldVelocity ile lead angle da hesaplanır → hareket halinde çalışır
            shooter.updateAiming(pose, speeds, true);
        } else {
            // Menzil dışı: rest pozisyonu
            shooter.stopFlywheel();
            shooter.setHoodAngle(30.0);  // mekanik minimum (rest)
            shooter.setTurretAngle(0.0); // merkez
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Sürekli çalışır
    }

    @Override
    public void end(boolean interrupted) {
        // Başka bir komut devralıyor — motora dokunmuyoruz,
        // ShootCommand / HomeHoodCommand kendi işini yapacak.
    }
}

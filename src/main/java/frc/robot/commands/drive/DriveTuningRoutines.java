package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.Set;

/**
 * PedroPathing tarzı sürüş PID ayar rutinleri.
 *
 * <p>Her rutin, robotun o anki konumunu başlangıç noktası olarak alır
 * (Commands.defer ile). Robotu sahada herhangi bir yere koyup çalıştırabilirsiniz.
 *
 * <p>Kullanım:
 * <ol>
 *   <li>Elastic Dashboard üzerinden Tuning Chooser'dan bir rutin seç</li>
 *   <li>Autonomous modunda robotu etkinleştir</li>
 *   <li>Hareketi izle — hata varsa Drive/moduleDriveP, Drive/moduleTurnP veya
 *       Drive/PoseDriveP değerlerini dashboard'dan ayarla</li>
 *   <li>Devre dışı bırak, tekrar etkinleştir — değerler otomatik güncellenir</li>
 * </ol>
 */
public final class DriveTuningRoutines {

    private DriveTuningRoutines() {}

    /**
     * İleri-geri testi: distanceMeters kadar ileri gider, 0.5s bekler, başa döner.
     * Ayarlar: moduleDriveP, PoseDriveP/D
     * İzle: Robot tam olarak N metre gitti mi? Overshoot/undershoot var mı?
     */
    public static Command forwardBackward(DriveSubsystem drive, double distanceMeters) {
        return Commands.defer(() -> {
            Pose2d start = drive.getPose();
            double cos = start.getRotation().getCos();
            double sin = start.getRotation().getSin();
            Pose2d target = new Pose2d(
                    start.getX() + distanceMeters * cos,
                    start.getY() + distanceMeters * sin,
                    start.getRotation());
            return Commands.sequence(
                    new SimpleDriveToPose(drive, target),
                    Commands.waitSeconds(0.5),
                    new SimpleDriveToPose(drive, start));
        }, Set.of(drive));
    }

    /**
     * Yan kayma testi: distanceMeters kadar sola kayar, 0.5s bekler, başa döner.
     * Ayarlar: moduleDriveP (yatay), tekerlek hizalaması
     * İzle: Kayarken robot ileri/geri kayıyor mu?
     */
    public static Command strafeLeftRight(DriveSubsystem drive, double distanceMeters) {
        return Commands.defer(() -> {
            Pose2d start = drive.getPose();
            double cos = start.getRotation().getCos();
            double sin = start.getRotation().getSin();
            // Robot çerçevesinde sol vektörü: (-sin, cos)
            Pose2d target = new Pose2d(
                    start.getX() - distanceMeters * sin,
                    start.getY() + distanceMeters * cos,
                    start.getRotation());
            return Commands.sequence(
                    new SimpleDriveToPose(drive, target),
                    Commands.waitSeconds(0.5),
                    new SimpleDriveToPose(drive, start));
        }, Set.of(drive));
    }

    /**
     * Yerinde dönme testi: angleDegrees kadar döner, 0.5s bekler, başa döner.
     * Ayarlar: moduleTurnP, PoseRotP/D
     * İzle: Robot tam açıya döndü mü? Titreme veya gecikmeli yanıt var mı?
     */
    public static Command spinInPlace(DriveSubsystem drive, double angleDegrees) {
        return Commands.defer(() -> {
            Pose2d start = drive.getPose();
            Pose2d target = new Pose2d(
                    start.getTranslation(),
                    start.getRotation().plus(Rotation2d.fromDegrees(angleDegrees)));
            return Commands.sequence(
                    new SimpleDriveToPose(drive, target),
                    Commands.waitSeconds(0.5),
                    new SimpleDriveToPose(drive, start));
        }, Set.of(drive));
    }

    /**
     * Kare yol testi: sideMeters kenar uzunluğunda kare çizer, başa döner.
     * Robot tüm hareket boyunca başlangıç yönelimini korur (yandan kayarak döner).
     * Ayarlar: Tüm sürüş PID değerleri birlikte
     * İzle: Kare tamamlandıktan sonra robot tam başlangıç noktasına döndü mü?
     */
    public static Command squarePath(DriveSubsystem drive, double sideMeters) {
        return Commands.defer(() -> {
            Pose2d start = drive.getPose();
            double cos = start.getRotation().getCos();
            double sin = start.getRotation().getSin();
            // İleri vektörü: (cos, sin) | Sol vektörü: (-sin, cos)
            Pose2d p1 = new Pose2d( // İleri
                    start.getX() + sideMeters * cos,
                    start.getY() + sideMeters * sin,
                    start.getRotation());
            Pose2d p2 = new Pose2d( // İleri + Sol
                    start.getX() + sideMeters * cos - sideMeters * sin,
                    start.getY() + sideMeters * sin + sideMeters * cos,
                    start.getRotation());
            Pose2d p3 = new Pose2d( // Sadece Sol
                    start.getX() - sideMeters * sin,
                    start.getY() + sideMeters * cos,
                    start.getRotation());
            return Commands.sequence(
                    new SimpleDriveToPose(drive, p1),
                    new SimpleDriveToPose(drive, p2),
                    new SimpleDriveToPose(drive, p3),
                    new SimpleDriveToPose(drive, start));
        }, Set.of(drive));
    }
}

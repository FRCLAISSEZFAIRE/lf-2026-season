package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Joystick ile Field-Relative sürüş komutu.
 * Supplier pattern kullanarak test edilebilir ve modüler.
 */
public class DriveWithJoystick extends Command {

    private final DriveSubsystem driveSubsystem;

    // Joystick girdileri (Supplier olarak)
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotSpeedSupplier;

    private boolean lastInvertJoystick;
    private boolean lastInvertRotation;

    /**
     * @param driveSubsystem   Sürüş alt sistemi
     * @param xSpeedSupplier   İleri/Geri hız supplier'ı (-1 ile 1 arası)
     * @param ySpeedSupplier   Sol/Sağ hız supplier'ı (-1 ile 1 arası)
     * @param rotSpeedSupplier Dönüş hızı supplier'ı (-1 ile 1 arası)
     */
    public DriveWithJoystick(
            DriveSubsystem driveSubsystem,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSpeedSupplier) {

        this.driveSubsystem = driveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotSpeedSupplier = rotSpeedSupplier;

        addRequirements(driveSubsystem);

        // RIO Preferences'tan kayıtlı değerleri yükle
        boolean savedInvertJoystick = edu.wpi.first.wpilibj.Preferences.getBoolean("Drive/InvertJoystick", false);
        boolean savedInvertRotation = edu.wpi.first.wpilibj.Preferences.getBoolean("Drive/InvertRotation", true);

        // Dashboard'a koy
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Drive/InvertJoystick",
                savedInvertJoystick);
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Drive/InvertRotation",
                savedInvertRotation);

        this.lastInvertJoystick = savedInvertJoystick;
        this.lastInvertRotation = savedInvertRotation;
    }

    @Override
    public void execute() {
        // Dashboard'dan ters çevirme ayarlarını oku
        boolean invertJoystick = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Tuning/Drive/InvertJoystick",
                lastInvertJoystick);
        boolean invertRotation = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Tuning/Drive/InvertRotation",
                lastInvertRotation);

        // Değiştiyse RIO Preferences'a kaydet
        if (invertJoystick != lastInvertJoystick) {
            edu.wpi.first.wpilibj.Preferences.setBoolean("Drive/InvertJoystick", invertJoystick);
            lastInvertJoystick = invertJoystick;
        }
        if (invertRotation != lastInvertRotation) {
            edu.wpi.first.wpilibj.Preferences.setBoolean("Drive/InvertRotation", invertRotation);
            lastInvertRotation = invertRotation;
        }

        // 1. Joystick değerlerini al
        double rawX = xSpeedSupplier.getAsDouble();
        double rawY = ySpeedSupplier.getAsDouble();
        double rawRot = rotSpeedSupplier.getAsDouble();

        // 2. İttifak (Alliance) Odaklı Yön Ayarı (Kullanıcı İsteği: Mavi'de Invert gereksinimini kaldırmak)
        // Mavi'de Invert gerekiyor olduğu için varsayılan olarak eksi ile çarpıyoruz (Mavi'yi düzeltir).
        // Kırmızıda ise WPILib alan yönünden dolayı (sürücüye göre uzaklaşması için) tekrar çevirilmeli.
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

        if (isRed) {
            // Kırmızı İttifak: Sürücü duvarı 16.5m'de. İleri itmek (uzaklaşmak) için Alan X ekseninde -X gitmeliyiz.
            // Zaten RobotContainer'da veya yukarıda algılanan işaretin tersine ihtiyacımız var.
            // Standardı sağlamak adına Kırmızı için X ve Y'yi düzelt.
            // Zaten rawX = rawX olduğu için koda bir atama yapmaya gerek yok, linter uyarısını engelliyoruz.
        } else {
            // Mavi İttifak varsayılanı (Eskiden Invert açmak gerekiyordu, artık otomatik invert)
            rawX = -rawX;
            rawY = -rawY;
        }

        // Manuel Ters Çevirme (Dashboard override)
        if (invertJoystick) {
            rawX = -rawX;
            rawY = -rawY;
        }
        if (invertRotation) {
            rawRot = -rawRot;
        }

        double xSpeed = MathUtil.applyDeadband(rawX, OIConstants.kDriveDeadband);
        double ySpeed = MathUtil.applyDeadband(rawY, OIConstants.kDriveDeadband);
        double rotSpeed = MathUtil.applyDeadband(rawRot, OIConstants.kDriveDeadband);

        // Squared Sensitivity for Rotation (daha hassas dönüş kontrolü)
        rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

        // 2. Hızları ölçeklendir
        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        rotSpeed *= DriveConstants.kMaxAngularSpeedRadPerSec;

        Logger.recordOutput("Drive/CommandedVX", xSpeed);
        Logger.recordOutput("Drive/CommandedVY", ySpeed);
        Logger.recordOutput("Drive/CommandedOmega", rotSpeed);

        // 3. Sürüş Mantığı
        if (xSpeed == 0 && ySpeed == 0 && rotSpeed == 0) {
            // Joystick bırakıldığında ANINDA dur ve X-stance pozisyonuna geç (Tekerleri
            // kilitle)
            driveSubsystem.stop();
        } else {
            // Saha-referanslı sürüş uygula (ESKİ MANUEL MOD)
            // Kullanıcının istediği referans koda uygun mantık
            driveSubsystem.driveManual(
                    xSpeed,
                    ySpeed,
                    rotSpeed,
                    true // fieldRelative
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Default command olarak sürekli çalışır
    }
}

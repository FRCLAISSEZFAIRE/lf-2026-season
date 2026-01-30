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
    }

    @Override
    public void execute() {
        // 1. Joystick değerlerini al ve deadband uygula
        double rawX = xSpeedSupplier.getAsDouble();
        double rawY = ySpeedSupplier.getAsDouble();
        double rawRot = rotSpeedSupplier.getAsDouble();

        double xSpeed = MathUtil.applyDeadband(rawX, OIConstants.kDriveDeadband);
        double ySpeed = MathUtil.applyDeadband(rawY, OIConstants.kDriveDeadband);
        double rotSpeed = MathUtil.applyDeadband(rawRot, OIConstants.kDriveDeadband);

        // DEBUG: Joystick değerlerini logla
        Logger.recordOutput("Drive/RawX", rawX);
        Logger.recordOutput("Drive/RawY", rawY);
        Logger.recordOutput("Drive/RawRot", rawRot);

        // Squared Sensitivity for Rotation (daha hassas dönüş kontrolü)
        rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

        Logger.recordOutput("Drive/AfterDeadbandX", xSpeed);
        Logger.recordOutput("Drive/AfterDeadbandY", ySpeed);
        Logger.recordOutput("Drive/AfterDeadbandRot", rotSpeed);

        // 2. Hızları ölçeklendir
        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        rotSpeed *= DriveConstants.kMaxAngularSpeedRadPerSec;

        Logger.recordOutput("Drive/CommandedVX", xSpeed);
        Logger.recordOutput("Drive/CommandedVY", ySpeed);
        Logger.recordOutput("Drive/CommandedOmega", rotSpeed);

        // 3. Drive Logic
        if (xSpeed == 0 && ySpeed == 0 && rotSpeed == 0) {
            // Joystick bırakıldığında ANINDA dur ve X-stance pozisyonuna geç (Tekerleri
            // kilitle)
            driveSubsystem.stop();
        } else {
            // Field-relative sürüş uygula (LEGACY MANUAL MODE)
            // Kullanıcının istediği referans koda uygun logic
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

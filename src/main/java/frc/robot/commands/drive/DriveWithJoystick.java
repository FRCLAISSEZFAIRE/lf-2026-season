package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.getAsDouble(), OIConstants.kDriveDeadband);
        double ySpeed = MathUtil.applyDeadband(ySpeedSupplier.getAsDouble(), OIConstants.kDriveDeadband);
        double rotSpeed = MathUtil.applyDeadband(rotSpeedSupplier.getAsDouble(), OIConstants.kDriveDeadband);

        // 2. Hızları ölçeklendir
        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        rotSpeed *= DriveConstants.kMaxAngularSpeedRadPerSec;

        // 3. Field-relative sürüş uygula
        driveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        rotSpeed,
                        driveSubsystem.getPose().getRotation()));
    }

    @Override
    public boolean isFinished() {
        return false; // Default command olarak sürekli çalışır
    }
}

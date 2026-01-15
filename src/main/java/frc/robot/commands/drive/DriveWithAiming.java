package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Joystick ile translasyon (hareket) yaparken,
 * robotun burnunu otomatik olarak hedefe çeviren komut.
 */
public class DriveWithAiming extends Command {

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final Supplier<Translation2d> targetLocationSupplier;

    // Dönüş PID Kontrolcüsü (Robot Heading için)
    private final PIDController rotationPID = new PIDController(
            DriveConstants.kTurnP,
            DriveConstants.kTurnI,
            DriveConstants.kTurnD);

    /**
     * @param driveSubsystem   Sürüş alt sistemi
     * @param xSpeedSupplier   İleri/Geri hız supplier'ı
     * @param ySpeedSupplier   Sol/Sağ hız supplier'ı
     * @param targetLocationSupplier Hedef konumu (Shooter'dan veya FieldConstants'tan alınabilir)
     */
    public DriveWithAiming(
            DriveSubsystem driveSubsystem,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            Supplier<Translation2d> targetLocationSupplier) {

        this.driveSubsystem = driveSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.targetLocationSupplier = targetLocationSupplier;

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationPID.setTolerance(Math.toRadians(2.0)); // 2 derece tolerans

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // 1. Joystick translation değerlerini al
        double xSpeed = MathUtil.applyDeadband(xSpeedSupplier.getAsDouble(), OIConstants.kDriveDeadband);
        double ySpeed = MathUtil.applyDeadband(ySpeedSupplier.getAsDouble(), OIConstants.kDriveDeadband);

        xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;

        // 2. Hedefe dönmek için açısal hızı hesapla
        Pose2d robotPose = driveSubsystem.getPose();
        Translation2d target = targetLocationSupplier.get();

        double targetAngleRad = Math.atan2(
                target.getY() - robotPose.getY(),
                target.getX() - robotPose.getX());

        double currentRotationRad = robotPose.getRotation().getRadians();
        
        // PID Çıktısı (Açısal Hız)
        double rotSpeed = rotationPID.calculate(currentRotationRad, targetAngleRad);
        
        // Clamp (Max dönüş hızını aşma)
        rotSpeed = MathUtil.clamp(rotSpeed, -DriveConstants.kMaxAngularSpeedRadPerSec, DriveConstants.kMaxAngularSpeedRadPerSec);

        // 3. Drive uygula
        driveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed,
                        ySpeed,
                        rotSpeed,
                        driveSubsystem.getPose().getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        // Komut bittiğinde durdurmaya gerek yok, çünkü default command devreye girecek
    }
}

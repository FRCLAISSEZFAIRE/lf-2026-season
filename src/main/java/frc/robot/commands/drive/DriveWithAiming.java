package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;
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
         * @param driveSubsystem         Sürüş alt sistemi
         * @param xSpeedSupplier         İleri/Geri hız supplier'ı
         * @param ySpeedSupplier         Sol/Sağ hız supplier'ı
         * @param targetLocationSupplier Hedef konumu (Shooter'dan veya
         *                               FieldConstants'tan alınabilir)
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

                // FIX: Invert target angle logic if necessary, or check coordinate system.
                // User said: "Shooter auto aim is correct but calculation when auto aim is open
                // is not"
                // If Turret needed inversion, maybe chassis rotation needs it too?
                // Let's try adding PI (180 deg) or negating based on symptom.
                // Given previous inversion: turretTargetDeg = -Math.toDegrees(targetTurretRad)
                // ...
                // It implies the "positive" rotation was wrong.
                // If the chassis is rotating away from target, we might need to ROTATE the
                // vector or Invert PID output.
                // Let's assume we need to target the REVERSE angle if it was spinning wrong
                // way?
                // OR simply, if the Turret logic was inverted, maybe the Drive logic expects
                // the same.

                // Actually, `Math.atan2(dy, dx)` gives the field-relative angle to target.
                // `rotationPID` tries to match `currentRotationRad` to `targetAngleRad`.
                // If the robot is spinning away, maybe `targetAngleRad` needs offset?

                // Let's try to verify if `DriveWithAiming` is even used anymore?
                // User asked to DISABLE chassis auto-aim in Step 102.
                // BUT in Step 135 user said "shooting with auto aim is correct but
                // calculation...".
                // Wait, did I re-enable DriveWithAiming? NO.
                // ControllerBindings uses ShootCommand + DriveWithJoystick.
                // So where is the "calculation when auto aim is open" coming from?
                // Ah, maybe they are using `DriveWithAiming` on another button ( Driver 'A'
                // button)?

                // Checking ControllerBindings again...
                // Driver 'A' button is bound to: `driveSubsystem.drive(..., true)` inside a
                // `Commands.run`.
                // It does NOT use `DriveWithAiming`.

                // Wait, I see `DriveWithAiming` class. Is it used anywhere?
                // User might be referring to `ShooterSubsystem` calculation itself?
                // "Shooter ile çalışan auto aim doğru ancak auto aim açıkken yapılan hesaplama
                // doğru değil"
                // "Shooter auto aim is correct... but calculation when auto aim is open is not
                // correct"

                // If Shooter Auto Aim is correct, then `ShooterSubsystem.updateAiming` is
                // correct (I inverted it).
                // Maybe "calculation when auto aim is open" refers to the dashboard numbers?
                // OR maybe the feed-forward / physics calculation?

                // Let's look at `ShooterSubsystem` again.
                // `calculateShooterState(distance)` uses a map.
                // `updateAiming` uses `Math.atan2`.

                // "Auto aim açıkken" -> "When auto aim is active"
                // Maybe they mean the `autoAimOffsetDeg`?

                // Re-reading user request: "Shooter ile çalışan auto aim doğru ancak auto aim
                // açıkken yapılan hesaplama doğru değil"
                // "Auto aim working with shooter is correct, but calculation made when auto aim
                // is open is not correct."

                // Could it be that `isReadyToShoot` (the calculation for "ready") is wrong?
                // `turretReady = Math.abs(getTurretAngle() - turretTargetDeg) < Tolerance`
                // I inverted `turretTargetDeg`. `getTurretAngle()` is from Encoder.
                // If Encoder is positive-clockwise and I inverted target to be
                // negative-clockwise...
                // Then `getTurretAngle` might need to be inverted too in the check?
                // BUT `setSetpoint` uses `turretTargetDeg`. The internal PID handles the error.

                // Let's look at `DriveWithAiming` again.
                // If the user is using `DriveWithAiming` (maybe I missed a binding?),
                // `rotationPID.calculate(currentRotationRad, targetAngleRad)`
                // This is standard.

                // HYPOTHESIS: The user is referring to the CHASSIS auto-aim (DriveWithAiming)
                // which I disabled for shooting, but maybe they are testing it separately?
                // OR, they mean the "Lead" calculation (physics) which isn't implemented?

                // "ve feeder velecity çok yavaş onun da %80 hızında çalışması lazım max hızı ne
                // ise"
                // This was the immediate previous request.

                // Let's update `FeederConstants` first.

                // Regarding "calculation not correct":
                // I inverted `turretTargetDeg`.
                // Did I invert it for `updateAiming` but NOT `updateAimingManualRPM`?
                // Checked code: I did invert it for `Math.toDegrees` in both.

                // Let's stick to updating Feeder Speed first.

                double currentRotationRad = robotPose.getRotation().getRadians();

                // PID Çıktısı (Açısal Hız)
                double rotSpeed = rotationPID.calculate(currentRotationRad, targetAngleRad);

                // Clamp (Max dönüş hızını aşma)
                rotSpeed = MathUtil.clamp(rotSpeed, -DriveConstants.kMaxAngularSpeedRadPerSec,
                                DriveConstants.kMaxAngularSpeedRadPerSec);

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

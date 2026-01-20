package frc.robot.commands.climber;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Otomatik Tırmanma Komutu.
 * 
 * <p>
 * ADIMLAR:
 * </p>
 * <ol>
 * <li>Seçilen Tower pozisyonuna git (Pathfinding)</li>
 * <li>Kancaları uzat (Extend)</li>
 * <li>Yavaşça ileri sür (Hizalama - Kancalar barın üzerine geçsin diye)</li>
 * <li>Robotu yukarı çek (Retract)</li>
 * <li>Kilitlenme sensörünü (Seat Sensor) bekle</li>
 * </ol>
 */
public class AutoClimbCommand extends SequentialCommandGroup {

        public AutoClimbCommand(
                        ClimberSubsystem climberSubsystem,
                        DriveSubsystem driveSubsystem,
                        Supplier<Pose2d> targetPoseSupplier) {

                addCommands(
                                // 1. Hedefe Git
                                Commands.defer(() -> {
                                        Pose2d target = targetPoseSupplier.get();
                                        return AutoBuilder.pathfindToPose(
                                                        target,
                                                        new PathConstraints(
                                                                        DriveConstants.kMaxSpeedMetersPerSecond * 0.5, // Daha
                                                                                                                       // yavaş
                                                                                                                       // yaklaş
                                                                        DriveConstants.kMaxSpeedMetersPerSecond * 0.5,
                                                                        DriveConstants.kMaxAngularSpeedRadPerSec * 0.5,
                                                                        DriveConstants.kMaxAngularSpeedRadPerSec * 0.5),
                                                        0.0 // Dur
                                        );

                                }, java.util.Set.of(driveSubsystem)), // DriveSubsystem gereksinimi

                                // 2. Kancaları Uzat
                                new ClimberExtendCommand(climberSubsystem),

                                // 3. Hafifçe İleri Sür (Kancayı bara takmak için)
                                // 0.5 saniye boyunca %10 hızla ileri
                                Commands.run(() -> driveSubsystem.drive(0.2, 0, 0, false, false), driveSubsystem)
                                                .withTimeout(0.5)
                                                .andThen(Commands.runOnce(() -> driveSubsystem.stop(), driveSubsystem)),

                                // 4. Yukarı Çek (Tırman)
                                new ClimberRetractCommand(climberSubsystem),

                                // 5. Sensör onayı bekle (Güvenlik)
                                Commands.waitUntil(climberSubsystem::isSeated));
        }
}

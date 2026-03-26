package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.commands.drive.SimpleDriveToPose;
import java.util.Set;

/**
 * FeedPassCommand
 * Robotun konumuna ve ittifak rengine göre FeedStart noktasından FeedStop
 * noktasını bulup
 * (veya tam tersi en yakından başlayarak) oraya doğru gider.
 * Bu gidiş esnasında Intake dışarı uzatılır (deploy) ve Roller çalıştırılır.
 * Hareket bittiğinde (veya iptal edildiğinde) intake toplanır ve roller
 * durdurulur.
 */
public class FeedPassCommand extends DeferredCommand {

    public FeedPassCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        super(() -> buildFeedCommand(driveSubsystem, intakeSubsystem),
                Set.of(driveSubsystem, intakeSubsystem));
    }

    private static Command buildFeedCommand(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        Pose2d currentPose = driveSubsystem.getPose();
        var alliance = DriverStation.getAlliance();
        boolean isTopHalf = currentPose.getY() > 4.0;

        Pose2d startPoint;
        Pose2d stopPoint;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // KIRMIZI İTTİFAK
            if (isTopHalf) {
                startPoint = FieldConstants.getFeedStopRedRight();
                stopPoint = FieldConstants.getFeedStartRedRight();
            } else {
                startPoint = FieldConstants.getFeedStopRedLeft();
                stopPoint = FieldConstants.getFeedStartRedLeft();

            }
        } else {
            // MAVİ İTTİFAK (Varsayılan)
            if (isTopHalf) {
                startPoint = FieldConstants.getFeedStopBlueLeft();
                stopPoint = FieldConstants.getFeedStartBlueLeft();
            } else {
                startPoint = FieldConstants.getFeedStopBlueRight();
                stopPoint = FieldConstants.getFeedStartBlueRight();
            }
        }

        // --- DEFENSIVE CHECKS ---
        if (startPoint == null || stopPoint == null) {
            System.err.println("[FeedPassCommand] Start or Stop point is NULL! Alliance: "
                    + (alliance.isPresent() ? alliance.get() : "None"));
            return Commands.none();
        }

        System.out.println("[FeedPassCommand] Targets found: Start=" + startPoint + ", Stop=" + stopPoint);

        // Önce YAKIN noktaya git, sonra uzak noktaya
        double distStart = currentPose.getTranslation().getDistance(startPoint.getTranslation());
        double distStop = currentPose.getTranslation().getDistance(stopPoint.getTranslation());

        Command driveSequence;
        if (distStart <= distStop) {
            System.out.println("[FeedPassCommand] Going to Start first, then Stop.");
            driveSequence = new SimpleDriveToPose(driveSubsystem, startPoint)
                    .andThen(new SimpleDriveToPose(driveSubsystem, stopPoint));
        } else {
            System.out.println("[FeedPassCommand] Going to Stop first, then Start.");
            driveSequence = new SimpleDriveToPose(driveSubsystem, stopPoint)
                    .andThen(new SimpleDriveToPose(driveSubsystem, startPoint));
        }

        // Feed modu: yavaş sürüş çarpanını aktifleştir
        // Intake aç, sürüş tamamlanana kadar roller çalışsın. Bitince topla (retract).
        return Commands.sequence(
                Commands.runOnce(() -> driveSubsystem.setFeedMode(true)),
                Commands.deadline(
                        driveSequence,
                        Commands.sequence(
                                intakeSubsystem.deployCommand(),
                                Commands.run(() -> intakeSubsystem.runRollerRPM(4800), intakeSubsystem))))
                .finallyDo((interrupted) -> {
                    driveSubsystem.setFeedMode(false);
                    intakeSubsystem.setExtensionPosition(IntakeConstants.kExtensionRetractedCm);
                    intakeSubsystem.stopRoller();
                });
    }
}

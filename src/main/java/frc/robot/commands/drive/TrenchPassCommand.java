package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.Set;

/**
 * TrenchPassCommand
 * Robotun bulunduğu konuma ve ittifak rengine göre otomatik olarak
 * A'dan B'ye veya B'den A'ya (Kırmızı için C'den D'ye veya D'den C'ye)
 * gitmesini sağlayan sıralı komut grubunu dinamik olarak oluşturur,
 * aynı zamanda komut başlarken Hood açısını 0'a konumlar.
 */
public class TrenchPassCommand extends DeferredCommand {

    public TrenchPassCommand(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
        super(() -> Commands.parallel(
                buildCommand(driveSubsystem),
                Commands.runOnce(() -> shooterSubsystem.setHoodAngle(0.0), shooterSubsystem)),
                Set.of(driveSubsystem, shooterSubsystem));
    }

    private static Command buildCommand(DriveSubsystem driveSubsystem) {
        Pose2d currentPose = driveSubsystem.getPose();
        var alliance = DriverStation.getAlliance();
        boolean isTopHalf = currentPose.getY() > 4.0;

        Pose2d pointA;
        Pose2d pointB;

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // RED ALLIANCE
            if (isTopHalf) {
                pointA = FieldConstants.getTransitionPointRedA();
                pointB = FieldConstants.getTransitionPointRedB();
            } else {
                pointA = FieldConstants.getTransitionPointRedC();
                pointB = FieldConstants.getTransitionPointRedD();
            }
        } else {
            // BLUE ALLIANCE (Default)
            if (isTopHalf) {
                pointA = FieldConstants.getTransitionPointBlueC();
                pointB = FieldConstants.getTransitionPointBlueD();
            } else {
                pointA = FieldConstants.getTransitionPointBlueA();
                pointB = FieldConstants.getTransitionPointBlueB();
            }
        }

        // Go to the CLOSER point first, then to the farther one
        double distA = currentPose.getTranslation().getDistance(pointA.getTranslation());
        double distB = currentPose.getTranslation().getDistance(pointB.getTranslation());

        if (distA <= distB) {
            return new SimpleDriveToPose(driveSubsystem, pointA)
                    .andThen(new SimpleDriveToPose(driveSubsystem, pointB));
        } else {
            return new SimpleDriveToPose(driveSubsystem, pointB)
                    .andThen(new SimpleDriveToPose(driveSubsystem, pointA));
        }
    }
}

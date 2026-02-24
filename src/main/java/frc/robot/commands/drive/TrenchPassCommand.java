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

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            // RED ALLIANCE
            if (isTopHalf) {
                // Red A and B are on the top side (Y > 4.0)
                if (currentPose.getX() > FieldConstants.kRedAllianceZoneMinX) {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedA())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedB()));
                } else {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedB())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedA()));
                }
            } else {
                // Red C and D are on the bottom side (Y < 4.0)
                if (currentPose.getX() > FieldConstants.kRedAllianceZoneMinX) {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedC())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedD()));
                } else {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedD())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointRedC()));
                }
            }
        } else {
            // BLUE ALLIANCE (Default)
            if (isTopHalf) {
                // Blue C and D are on the top side (Y > 4.0)
                if (currentPose.getX() < FieldConstants.kBlueAllianceZoneMaxX) {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueC())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueD()));
                } else {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueD())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueC()));
                }
            } else {
                // Blue A and B are on the bottom side (Y < 4.0)
                if (currentPose.getX() < FieldConstants.kBlueAllianceZoneMaxX) {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueA())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueB()));
                } else {
                    return new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueB())
                            .andThen(new SimpleDriveToPose(driveSubsystem, FieldConstants.getTransitionPointBlueA()));
                }
            }
        }
    }
}

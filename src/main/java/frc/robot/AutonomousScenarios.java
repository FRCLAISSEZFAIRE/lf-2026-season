package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.drive.BumpPassCommand;
import frc.robot.commands.drive.DriveTuningRoutines;
import frc.robot.commands.drive.SimpleDriveToPose;
import frc.robot.commands.drive.TrenchPassCommand;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.TunableNumber;
import java.util.Set;

/**
 * Defines autonomous scenarios.
 *
 * <p>
 * Dashboard Choosers:
 * </p>
 * <ul>
 * <li>Auto Chooser - scenario selection</li>
 * <li>Pass Mode - Trench or Bump</li>
 * <li>Start Side - Left or Right (determines which pass route)</li>
 * </ul>
 */
public final class AutonomousScenarios {

        private static final TunableNumber fullShootTimeout = new TunableNumber("Auto", "FullShootTimeout", 5.0);
        private static final SendableChooser<String> passChooser = new SendableChooser<>();
        private static final SendableChooser<String> sideChooser = new SendableChooser<>();

        private AutonomousScenarios() {
        }

        public static SendableChooser<Command> buildChooser(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {
                SendableChooser<Command> chooser = new SendableChooser<>();
                chooser.setDefaultOption("0 - Do Nothing", Commands.none());
                chooser.addOption("1 - Collect and Shoot", collectAndShoot(drive, shooter, feeder, intake));
                chooser.addOption("2 - Source, Shoot", sourceAndShoot(drive, shooter, feeder, intake));
                chooser.addOption("3 - Double Collect & Shoot", doubleCollectAndShoot(drive, shooter, feeder, intake));
                // --- Tuning Rutinleri ---
                chooser.addOption("T1 - İleri/Geri 1m", DriveTuningRoutines.forwardBackward(drive, 1.0));
                chooser.addOption("T2 - İleri/Geri 2m", DriveTuningRoutines.forwardBackward(drive, 2.0));
                chooser.addOption("T3 - Yan Kayma 1m", DriveTuningRoutines.strafeLeftRight(drive, 1.0));
                chooser.addOption("T4 - Yerinde Dönme 90°", DriveTuningRoutines.spinInPlace(drive, 90));
                chooser.addOption("T5 - Yerinde Dönme 180°", DriveTuningRoutines.spinInPlace(drive, 180));
                chooser.addOption("T6 - Kare Yol 1m", DriveTuningRoutines.squarePath(drive, 1.0));

                passChooser.setDefaultOption("Trench", "Trench");
                passChooser.addOption("Bump", "Bump");
                SmartDashboard.putData("Pass/Pass Mode", passChooser);

                sideChooser.setDefaultOption("Left", "Left");
                sideChooser.addOption("Right", "Right");
                SmartDashboard.putData("Auto/Start Side", sideChooser);

                SmartDashboard.putData("Auto Chooser", chooser);
                return chooser;
        }

        private static Command collectAndShoot(DriveSubsystem drive, ShooterSubsystem shooter, FeederSubsystem feeder,
                        IntakeSubsystem intake) {
                return Commands.sequence(getMainPassCommand(drive, shooter),
                                autoIntakeToManualPoint(drive, intake, feeder, true),
                                new BumpPassCommand(drive, shooter), fullShoot(shooter, feeder, drive, intake));
        }

        private static Command sourceAndShoot(DriveSubsystem drive, ShooterSubsystem shooter, FeederSubsystem feeder,
                        IntakeSubsystem intake) {
                return Commands.sequence(
                                Commands.deadline(
                                                new DeferredCommand(() -> new SimpleDriveToPose(drive,
                                                                FieldConstants.getSourcePose(
                                                                                DriverStation.getAlliance())),
                                                                Set.<Subsystem>of(drive)),
                                                Commands.runEnd(() -> {
                                                        intake.deploy();
                                                        intake.runRollerRPM(IntakeConstants.kRollerTargetRPM);
                                                        feeder.feed();
                                                }, () -> {
                                                        intake.stopRoller();
                                                        feeder.stop();
                                                }, intake, feeder)),
                                Commands.parallel(Commands.runEnd(() -> {
                                        intake.deploy();
                                        intake.runRollerRPM(IntakeConstants.kRollerTargetRPM);
                                }, () -> {
                                        intake.stopRoller();
                                }, intake),
                                                new AutoShootCommand(shooter, feeder, drive, null, drive::getPose)));
        }

        private static Command doubleCollectAndShoot(DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {
                return Commands.sequence(
                                getMainPassCommand(drive, shooter),
                                autoIntakeToManualPoint(drive, intake, feeder, true),
                                new BumpPassCommand(drive, shooter), fullShoot(shooter, feeder, drive, intake),
                                getMainPassCommand(drive, shooter),
                                autoIntakeToManualPoint(drive, intake, feeder, false),
                                new BumpPassCommand(drive, shooter), fullShoot(shooter, feeder, drive, intake));
        }

        /**
         * Scenario 4: Waypoint Test - Sequential move through field points.
         * 
         * <p>
         * Units:
         * </p>
         * <ul>
         * <li><b>Pose2d (X, Y):</b> Metre cinsinden saha koordinatları (0-16.5m). Sol
         * alt köşe (0,0)'dır.</li>
         * <li><b>Rotation2d:</b> Derece cinsinden robot yönelimi (0° = Blue Alliance
         * ileri, 180° = Red Alliance ileri).</li>
         * <li><b>Passing Tolerance:</b> Metre cinsinden geçiş toleransı. Robot bu
         * mesafeye girdiğinde durmadan bir sonrakine geçer.</li>
         * </ul>
         * 
         * <p>
         * Behavior:
         * </p>
         * <ul>
         * <li>Noktalar arasında "Automatic Curve" (1.5m kavis) ile yumuşak geçiş
         * yapar.</li>
         * <li>Belirtilen noktalarda (örn. 7m, 2m) paralel komutlar (Intake Deploy)
         * tetikleyebilir.</li>
         * <li>Yasaklı bölgelere (Forbidden Zones) girmeyi reddeder veya buralara
         * girerken hızı keser.</li>
         * </ul>
         * 
         * @param drive  Sürüş alt sistemi.
         * @param intake Intake alt sistemi.
         * @return Waypoint takip komutu.
         */

        public static Command getMainPassCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
                return new DeferredCommand(() -> {
                        String selection = passChooser.getSelected();
                        return "Bump".equals(selection) ? new BumpPassCommand(drive, shooter)
                                        : new TrenchPassCommand(drive, shooter);
                }, Set.<Subsystem>of(drive, shooter));
        }

        private static Command fullShoot(ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive,
                        IntakeSubsystem intake) {
                return new AutoShootCommand(shooter, feeder, drive, intake, drive::getPose);
        }

        private static Command autoIntakeToManualPoint(DriveSubsystem drive, IntakeSubsystem intake,
                        FeederSubsystem feeder, boolean isRound1) {
                return Commands.deadline(new DeferredCommand(() -> {
                        Translation2d point = getManualReturnPoint(drive, isRound1);
                        Translation2d currentPos = drive.getPose().getTranslation();
                        Rotation2d targetAngle = new Rotation2d(point.getX() - currentPos.getX(),
                                        point.getY() - currentPos.getY());
                        return new SimpleDriveToPose(drive, new Pose2d(currentPos, targetAngle))
                                        .andThen(new SimpleDriveToPose(drive, new Pose2d(point, targetAngle)));
                }, Set.<Subsystem>of(drive)),
                                Commands.runEnd(() -> {
                                        intake.setExtensionPosition(IntakeConstants.kExtensionDeployedCm);
                                        intake.runRollerRPM(IntakeConstants.kRollerTargetRPM);
                                }, () -> {
                                        intake.stopRoller();
                                }, intake, feeder));
        }

        private static Translation2d getManualReturnPoint(DriveSubsystem drive, boolean isRound1) {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
                boolean isLeft = isRed ? drive.getPose().getY() <= 4.0 : drive.getPose().getY() >= 4.0;
                if (isRed) {
                        return isLeft ? (isRound1 ? new Translation2d(8.8, 2.6) : new Translation2d(8, 4))
                                        : (isRound1 ? new Translation2d(8.8, 5.6) : new Translation2d(8, 4));
                } else {
                        return isLeft ? (isRound1 ? new Translation2d(7.8, 5.7) : new Translation2d(8, 4))
                                        : (isRound1 ? new Translation2d(7.8, 2.4) : new Translation2d(8, 4));
                }
        }
}

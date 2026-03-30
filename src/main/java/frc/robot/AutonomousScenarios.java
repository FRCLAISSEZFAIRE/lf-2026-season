package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.drive.BumpPassCommand;
import frc.robot.commands.drive.SimpleDriveToPose;
import frc.robot.commands.drive.TrenchPassCommand;

import frc.robot.commands.shooter.ShootCommand;
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

        // ==========================================================================
        // TIMING SETTINGS - Adjustable from Dashboard, saved to RIO
        // ==========================================================================
        private static final TunableNumber fullShootTimeout = new TunableNumber("Auto", "FullShootTimeout", 5.0);

        private static final SendableChooser<String> passChooser = new SendableChooser<>();
        private static final SendableChooser<String> sideChooser = new SendableChooser<>();

        private AutonomousScenarios() {
        }

        /**
         * Creates the SendableChooser containing all autonomous scenarios.
         * Called from RobotContainer.
         */
        public static SendableChooser<Command> buildChooser(
                        DriveSubsystem drive,
                        ShooterSubsystem shooter,
                        FeederSubsystem feeder,
                        IntakeSubsystem intake) {

                SendableChooser<Command> chooser = new SendableChooser<>();

                chooser.setDefaultOption("0 - Do Nothing", Commands.none());

                chooser.addOption("1 - Collect and Shoot",
                                collectAndShoot(drive, shooter, feeder, intake));

                chooser.addOption("2 - Source, Shoot",
                                sourceAndShoot(drive, shooter, feeder, intake));

                chooser.addOption("3 - Double Collect & Shoot",
                                doubleCollectAndShoot(drive, shooter, feeder, intake));

                // Pass mode chooser
                passChooser.setDefaultOption("Trench", "Trench");
                passChooser.addOption("Bump", "Bump");
                SmartDashboard.putData("Pass/Pass Mode", passChooser);

                // Start side chooser (Left = top half Y>4, Right = bottom half Y<4)
                sideChooser.setDefaultOption("Left", "Left");
                sideChooser.addOption("Right", "Right");
                SmartDashboard.putData("Auto/Start Side", sideChooser);

                SmartDashboard.putData("Auto Chooser", chooser);

                return chooser;
        }

        // ==========================================================================
        // SCENARIO DEFINITIONS
        // ==========================================================================

        /**
         * Scenario 1: Pass -> autoIntake(to START) -> Pass back -> Shoot
         */
        private static Command collectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // Pass across
                                getMainPassCommand(drive, shooter),

                                // Bulunduğu konumdan (Trench bitişi) manuel belirlediğiniz ara noktaya doğru
                                // giderken toplama işlemi yapar
                                autoIntakeToManualPoint(drive, intake, feeder, true),

                                // Sonrasında gene bumpPass çalışacak
                                new BumpPassCommand(drive, shooter),

                                // Shoot
                                fullShoot(shooter, feeder, drive, intake));
        }

        /**
         * Scenario 2: Drive to Source (intake running) -> Arrive -> Intake + AutoShoot
         * parallel
         */
        private static Command sourceAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // 1. Drive to source while running intake + feeder
                                Commands.deadline(
                                                new DeferredCommand(() -> new SimpleDriveToPose(drive,
                                                                FieldConstants.getSourcePose(
                                                                                DriverStation.getAlliance())),
                                                                Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(
                                                                                drive)),
                                                Commands.runEnd(
                                                                () -> {
                                                                        intake.deploy();
                                                                        intake.runRollerRPM(2000);
                                                                        feeder.feed();
                                                                },
                                                                () -> {
                                                                        // Drive bitti, intake dur (sonraki adımda
                                                                        // tekrar açılacak)
                                                                        intake.stopRoller();
                                                                        feeder.stop();
                                                                },
                                                                intake, feeder)),
                                // 2. Vardıktan sonra: Intake deploy + roller + AutoShoot aynı anda
                                Commands.parallel(
                                                // Intake açık, roller çalışıyor
                                                Commands.runEnd(
                                                                () -> {
                                                                        intake.deploy();
                                                                        intake.runRollerRPM(2000); // Set edilecek
                                                                                                   // değerleri buraya
                                                                                                   // girebilirsiniz
                                                                },
                                                                () -> {
                                                                        intake.stopRoller();
                                                                        intake.retract();
                                                                },
                                                                intake),
                                                // AutoShoot: nişan al + ateş et (Intake'i karıştırmadan)
                                                new AutoShootCommand(shooter, feeder, drive, null,
                                                                drive::getPose)));
        }

        /**
         * Scenario 3: Pass -> autoIntake(to START) -> Pass back -> Shoot ->
         * Pass -> autoIntake(to STOP) -> Shoot
         */
        private static Command doubleCollectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // Round 1: pass across
                                getMainPassCommand(drive, shooter),
                                autoIntakeToManualPoint(drive, intake, feeder, true), // 1. tur toplama
                                new BumpPassCommand(drive, shooter), // BumpPass ile dönüş
                                fullShoot(shooter, feeder, drive, intake),

                                // Round 2: pass across
                                getMainPassCommand(drive, shooter),
                                autoIntakeToManualPoint(drive, intake, feeder, false), // 2. tur toplama
                                new BumpPassCommand(drive, shooter), // BumpPass ile dönüş
                                fullShoot(shooter, feeder, drive, intake));
        }

        // ==========================================================================
        // PASS COMMANDS
        // ==========================================================================

        /**
         * Returns the main pass command (Trench or Bump) based on Dashboard selection.
         */
        public static Command getMainPassCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
                return new DeferredCommand(() -> {
                        String selection = passChooser.getSelected();
                        if ("Bump".equals(selection)) {
                                return new BumpPassCommand(drive, shooter);
                        } else {
                                return new TrenchPassCommand(drive, shooter);
                        }
                }, Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive, shooter));
        }

        /**
         * Returns the alternative pass command (Trench or Bump) based on Dashboard
         * selection.
         */
        public static Command getAlternativePassCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
                return new DeferredCommand(() -> {
                        String selection = passChooser.getSelected();
                        if ("Bump".equals(selection)) {
                                return new TrenchPassCommand(drive, shooter);
                        } else {
                                return new BumpPassCommand(drive, shooter);
                        }
                }, Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive, shooter));
        }

        /**
         * Gets the appropriate pass point based on:
         * - Pass mode (Trench/Bump)
         * - Alliance (Red/Blue)
         * - Start side (Left/Right)
         * - isStart: true = first point (A/C), false = second point (B/D)
         */
        private static Pose2d getPassPoint(boolean isStart) {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
                boolean isLeft = "Left".equals(sideChooser.getSelected());
                boolean isTrench = !"Bump".equals(passChooser.getSelected());

                if (isTrench) {
                        if (isRed) {
                                // Red Left = A/B, Red Right = C/D
                                if (isLeft) {
                                        return isStart ? FieldConstants.getTrenchPointRedA()
                                                        : FieldConstants.getTrenchPointRedB();
                                } else {
                                        return isStart ? FieldConstants.getTrenchPointRedC()
                                                        : FieldConstants.getTrenchPointRedD();
                                }
                        } else {
                                // Blue Left = C/D, Blue Right = A/B
                                if (isLeft) {
                                        return isStart ? FieldConstants.getTrenchPointBlueC()
                                                        : FieldConstants.getTrenchPointBlueD();
                                } else {
                                        return isStart ? FieldConstants.getTrenchPointBlueA()
                                                        : FieldConstants.getTrenchPointBlueB();
                                }
                        }
                } else {
                        // Bump
                        if (isRed) {
                                if (isLeft) {
                                        return isStart ? FieldConstants.getBumpPointRedA()
                                                        : FieldConstants.getBumpPointRedB();
                                } else {
                                        return isStart ? FieldConstants.getBumpPointRedC()
                                                        : FieldConstants.getBumpPointRedD();
                                }
                        } else {
                                if (isLeft) {
                                        return isStart ? FieldConstants.getBumpPointBlueC()
                                                        : FieldConstants.getBumpPointBlueD();
                                } else {
                                        return isStart ? FieldConstants.getBumpPointBlueA()
                                                        : FieldConstants.getBumpPointBlueB();
                                }
                        }
                }
        }

        // ==========================================================================
        // ARA DÖNÜŞ NOKTALARI (MANUAL RETURN POINTS)
        // ==========================================================================

        /**
         * Toplamadan (collect) sonra dönüş öncesi gidilecek ara noktaları belirler.
         * Toplam 8 farklı senaryo (Mavi/Kırmızı, Sol/Sağ, 1.Tur/2.Tur) için X ve Y
         * (metre) değerlerini buradan girebilirsiniz.
         */
        private static edu.wpi.first.math.geometry.Translation2d getManualReturnPoint(DriveSubsystem drive,
                        boolean isRound1) {
                var alliance = DriverStation.getAlliance();
                boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;
                boolean isLeft;
                if (isRed) {
                        isLeft = drive.getPose().getY() <= 4.0; // Kırmızıda Y>4 Sağ, Y<=4 Sol
                } else {
                        isLeft = drive.getPose().getY() >= 4.0; // Mavide Y<4 Sağ, Y>=4 Sol
                }

                if (isRed) {
                        if (isLeft) {
                                return isRound1 ? new edu.wpi.first.math.geometry.Translation2d(8.8, 2.6) : // Kırmızı
                                                                                                            // Takım -
                                                                                                            // Sol Taraf
                                                                                                            // - 1. Tur
                                                new edu.wpi.first.math.geometry.Translation2d(8, 4); // Kırmızı
                                                                                                     // Takım - Sol
                                                                                                     // Taraf - 2.
                                                                                                     // Tur
                        } else {
                                return isRound1 ? new edu.wpi.first.math.geometry.Translation2d(8.8, 5.6) : // Kırmızı
                                                                                                            // Takım -
                                                                                                            // Sağ Taraf
                                                                                                            // - 1. Tur
                                                new edu.wpi.first.math.geometry.Translation2d(8, 4); // Kırmızı Takım -
                                                                                                     // Sağ Taraf - 2.
                                                                                                     // Tur
                        }
                } else {
                        if (isLeft) {
                                return isRound1 ? new edu.wpi.first.math.geometry.Translation2d(7.8, 5.7) : // Mavi
                                                                                                            // Takım -
                                                                                                            // Sol Taraf
                                                                                                            // - 1. Tur
                                                new edu.wpi.first.math.geometry.Translation2d(8, 4); // Mavi Takım - Sol
                                                                                                     // Taraf - 2. Tur
                        } else {
                                return isRound1 ? new edu.wpi.first.math.geometry.Translation2d(7.8, 2.4) : // Mavi
                                                                                                            // Takım -
                                                                                                            // Sağ Taraf
                                                                                                            // - 1. Tur
                                                new edu.wpi.first.math.geometry.Translation2d(8, 4); // Mavi Takım - Sağ
                                                                                                     // Taraf - 2. Tur
                        }
                }
        }

        // ==========================================================================
        // HELPER COMMANDS
        // ==========================================================================

        /** Full shoot - timeout adjustable from dashboard */
        private static Command fullShoot(
                        ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive,
                        IntakeSubsystem intake) {
                return new AutoShootCommand(shooter, feeder, drive, intake, drive::getPose);
        }

        /**
         * Drives to a MANUALLY set return point while running intake + feeder.
         */
        private static Command autoIntakeToManualPoint(
                        DriveSubsystem drive, IntakeSubsystem intake,
                        FeederSubsystem feeder, boolean isRound1) {
                return Commands.deadline(
                                // Bulunduğu konumdan manuel hedef noktasına aynı açı ile sür
                                new DeferredCommand(() -> {
                                        var point = getManualReturnPoint(drive, isRound1);

                                        // Robotun o anki konumu
                                        var currentPos = drive.getPose().getTranslation();

                                        // Hedefe olan açıyı (atan2 ile) hesapla
                                        double dx = point.getX() - currentPos.getX();
                                        double dy = point.getY() - currentPos.getY();
                                        var targetAngle = new edu.wpi.first.math.geometry.Rotation2d(dx, dy);

                                        // 1. Durduğu yerde önce yüzünü (başını) hedefe dönsün
                                        Pose2d turnPose = new Pose2d(currentPos, targetAngle);

                                        // 2. Ardından saptanan açı ile hedefin X,Y noktasına gitsin
                                        Pose2d drivePose = new Pose2d(point, targetAngle);

                                        System.out.println("[Auto] Intake turning to " + targetAngle.getDegrees()
                                                        + " then driving to " + point);

                                        return new SimpleDriveToPose(drive, turnPose)
                                                        .andThen(new SimpleDriveToPose(drive, drivePose));
                                }, Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive)),
                                // Run intake + feeder while driving
                                Commands.runEnd(
                                                () -> {
                                                        intake.setExtensionPosition(
                                                                        IntakeConstants.kExtensionDeployedCm);
                                                        intake.runRollerRPM(2000);
                                                },
                                                () -> {
                                                        intake.stopRoller();
                                                        intake.setExtensionPosition(
                                                                        IntakeConstants.kExtensionRetractedCm);
                                                },
                                                intake, feeder));
        }

}

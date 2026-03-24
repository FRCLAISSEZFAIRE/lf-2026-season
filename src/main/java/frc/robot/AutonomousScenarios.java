package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.drive.SimpleDriveToPose;
import frc.robot.commands.drive.TrenchPassCommand;
import frc.robot.commands.intake.FeedPassCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Otonom senaryolarını tanımlayan sınıf.
 * 
 * <p>
 * Yeni senaryo eklemek için:
 * 1. Aşağıya yeni bir private static metot ekle
 * 2. buildChooser() içinde addOption() ile kaydet
 * </p>
 * 
 * <p>
 * Süre ayarları Dashboard'dan düzenlenebilir:
 * Tuning/Auto/IntakeTimeout, StartShootTimeout, FullShootTimeout
 * </p>
 */
public final class AutonomousScenarios {

        // ==========================================================================
        // SÜRE AYARLARI — Dashboard'dan düzenlenebilir, RIO'ya kayıtlı
        // ==========================================================================
        private static final TunableNumber startShootTimeout = new TunableNumber("Auto", "StartShootTimeout", 2.0);
        private static final TunableNumber fullShootTimeout = new TunableNumber("Auto", "FullShootTimeout", 5.0);

        private AutonomousScenarios() {
        }

        /**
         * Tüm otonom senaryolarını içeren SendableChooser oluşturur.
         * RobotContainer'dan çağrılır.
         */
        public static SendableChooser<Command> buildChooser(
                        DriveSubsystem drive,
                        ShooterSubsystem shooter,
                        FeederSubsystem feeder,
                        IntakeSubsystem intake) {

                SendableChooser<Command> chooser = new SendableChooser<>();

                chooser.setDefaultOption("0 - Hiçbir Şey Yapma", Commands.none());

                chooser.addOption("1 - Topla Şut at",
                                collectAndShoot(drive, shooter, feeder, intake));

                chooser.addOption("2 - Topla, Outpost, Shoot",
                                collectShootAndClimb(drive, shooter, feeder, intake));

                chooser.addOption("3 - Çift Tur Topla & Şut",
                                doubleCollectAndShoot(drive, shooter, feeder, intake));

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Auto Chooser", chooser);

                return chooser;
        }

        // ==========================================================================
        // SENARYO TANIMLARI
        // ==========================================================================

        /**
         * Senaryo 1: TrenchPass -> autoIntake -> TrenchPass -> Şut at
         */
        private static Command collectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                intake.deployCommand(),
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                new TrenchPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive));
        }

        /**
         * Senaryo 2: TrenchPass -> autoIntake -> TrenchPass -> Outpost'a git -> Şut at
         */
        private static Command collectShootAndClimb(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                intake.deployCommand(),
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                new TrenchPassCommand(drive, shooter),
                                driveToOutpost(drive),
                                fullShoot(shooter, feeder, drive));
        }

        /**
         * Senaryo 3: TrenchPass -> autoIntake -> TrenchPass -> Şut -> TrenchPass -> autoIntake -> Şut
         */
        private static Command doubleCollectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                intake.deployCommand(),
                                // 1. Tur
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                new TrenchPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive),
                                // 2. Tur
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                fullShoot(shooter, feeder, drive));
        }

        // ==========================================================================
        // YARDIMCI KOMUTLAR
        // ==========================================================================

        /** Tam şut — süre dashboard'dan ayarlanabilir */
        private static Command fullShoot(
                        ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive) {
                return new ShootCommand(shooter, feeder, drive, drive::getPose)
                                .withTimeout(fullShootTimeout.get());
        }

        /** Başlangıç kısa şutu — süre dashboard'dan ayarlanabilir */
        private static Command startShoot(
                        ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive) {
                return new ShootCommand(shooter, feeder, drive, drive::getPose)
                                .withTimeout(startShootTimeout.get());
        }

        /** Top toplama — FeedStart→FeedStop noktalarına giderek intake çalıştırır */
        private static Command autoIntake(
                        DriveSubsystem drive, IntakeSubsystem intake, FeederSubsystem feeder) {
                return new FeedPassCommand(drive, intake);
        }

        /** Outpost noktasına sürüş komutu */
        private static Command driveToOutpost(DriveSubsystem drive) {
                return new DeferredCommand(() -> {
                        return new SimpleDriveToPose(drive, FieldConstants.getOutpostPose(DriverStation.getAlliance()));
                }, java.util.Set.of(drive));
        }

    
}

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.commands.drive.BumpPassCommand;
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
        private static final TunableNumber fullShootTimeout = new TunableNumber("Auto", "FullShootTimeout", 5.0);

        private static final SendableChooser<String> passChooser = new SendableChooser<>();

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

                passChooser.setDefaultOption("Trench", "Trench");
                passChooser.addOption("Bump", "Bump");
                SmartDashboard.putData("Pass/Pass Mode", passChooser);

                SmartDashboard.putData("Auto Chooser", chooser);

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
                                getMainPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                getMainPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive, intake));
        }

        /**
         * Senaryo 2: TrenchPass -> autoIntake -> TrenchPass -> Outpost'a git -> Şut at
         */
        private static Command collectShootAndClimb(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                getMainPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                getMainPassCommand(drive, shooter),
                                driveToOutpost(drive),
                                fullShoot(shooter, feeder, drive, intake));
        }

        /**
         * Senaryo 3: TrenchPass -> autoIntake -> TrenchPass -> Şut -> TrenchPass ->
         * autoIntake -> Şut
         */
        private static Command doubleCollectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // 1. Tur
                                getMainPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                getMainPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive, intake),
                                // 2. Tur
                                getMainPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                fullShoot(shooter, feeder, drive, intake));
        }

        /**
         * Ana geçiş komutunu (Trench veya Bump) Dashboard seçimine göre döndürür.
         */
        public static Command getMainPassCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
                return new DeferredCommand(() -> {
                        String selection = passChooser.getSelected();
                        if ("Bump".equals(selection)) {
                                return new BumpPassCommand(drive, shooter);
                        } else {
                                return new TrenchPassCommand(drive, shooter);
                        }
                }, java.util.Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive, shooter));
        }

        /**
         * Alternatif geçiş komutunu (Trench veya Bump) Dashboard seçimine göre
         * döndürür.
         */
        public static Command getAlternativePassCommand(DriveSubsystem drive, ShooterSubsystem shooter) {
                return new DeferredCommand(() -> {
                        String selection = passChooser.getSelected();
                        if ("Bump".equals(selection)) {
                                return new TrenchPassCommand(drive, shooter);
                        } else {
                                return new BumpPassCommand(drive, shooter);
                        }
                }, java.util.Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive, shooter));
        }

        // ==========================================================================
        // YARDIMCI KOMUTLAR
        // ==========================================================================

        /** Tam şut — süre dashboard'dan ayarlanabilir */
        private static Command fullShoot(
                        ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive,
                        IntakeSubsystem intake) {
                return new ShootCommand(shooter, feeder, drive, intake, drive::getPose)
                                .withTimeout(fullShootTimeout.get());
        }

        /** Top toplama — FeedStart→FeedStop noktalarına giderek intake çalıştırır */
        private static Command autoIntake(
                        DriveSubsystem drive, IntakeSubsystem intake, FeederSubsystem feeder) {
                return Commands.deadline(
                        new FeedPassCommand(drive, intake),
                        Commands.runEnd(() -> feeder.feed(), () -> feeder.stop(), feeder)
                );
        }

        /** Outpost noktasına sürüş komutu */
        private static Command driveToOutpost(DriveSubsystem drive) {
                return new DeferredCommand(() -> {
                        return new SimpleDriveToPose(drive, FieldConstants.getOutpostPose(DriverStation.getAlliance()));
                }, java.util.Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive));
        }

}

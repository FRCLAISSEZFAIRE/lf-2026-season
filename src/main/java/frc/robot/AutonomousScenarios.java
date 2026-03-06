package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.commands.climber.AutoClimbCommand;
import frc.robot.commands.drive.TrenchPassCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
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
        private static final TunableNumber intakeTimeout = new TunableNumber("Auto", "IntakeTimeout", 5.0);
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
                        IntakeSubsystem intake,
                        ClimberSubsystem climber) {

                SendableChooser<Command> chooser = new SendableChooser<>();

                chooser.setDefaultOption("0 - Hiçbir Şey Yapma", Commands.none());

                chooser.addOption("1 - Topla & Şut At",
                                collectAndShoot(drive, shooter, feeder, intake));

                chooser.addOption("2 - Topla, Şut & Tırman",
                                collectShootAndClimb(drive, shooter, feeder, intake, climber));

                chooser.addOption("3 - Çift Tur Topla & Şut",
                                doubleCollectAndShoot(drive, shooter, feeder, intake));

                edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Auto Chooser", chooser);

                return chooser;
        }

        // ==========================================================================
        // SENARYO TANIMLARI
        // ==========================================================================

        /**
         * Senaryo 1: Başlangıç şutu → Top topla → Alana geçiş → Şut at
         */
        private static Command collectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                startShoot(shooter, feeder, drive),
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                new TrenchPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive));
        }

        /**
         * Senaryo 2: Başlangıç şutu → Top topla → Alana geçiş → Şut at → Tırman
         */
        private static Command collectShootAndClimb(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake,
                        ClimberSubsystem climber) {

                return Commands.sequence(
                                startShoot(shooter, feeder, drive),
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                new TrenchPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive),
                                autoClimb(climber, drive));
        }

        /**
         * Senaryo 3: Başlangıç şutu → (Top topla → Geçiş → Şut) x2
         */
        private static Command doubleCollectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // 1. Tur
                                startShoot(shooter, feeder, drive),
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                new TrenchPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive),
                                // 2. Tur
                                new TrenchPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                new TrenchPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive));
        }

        // ==========================================================================
        // YARDIMCI KOMUTLAR
        // ==========================================================================

        /** Tam şut — süre dashboard'dan ayarlanabilir */
        private static Command fullShoot(
                        ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive) {
                return new ShootCommand(shooter, feeder, drive::getPose)
                                .withTimeout(fullShootTimeout.get());
        }

        /** Başlangıç kısa şutu — süre dashboard'dan ayarlanabilir */
        private static Command startShoot(
                        ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive) {
                return new ShootCommand(shooter, feeder, drive::getPose)
                                .withTimeout(startShootTimeout.get());
        }

        /** Top toplama — belirtilen süre boyunca çalışır (saniye) */
        private static Command autoIntake(
                        DriveSubsystem drive, IntakeSubsystem intake, FeederSubsystem feeder) {
                return new AutoIntakeCommand(drive, intake, feeder, intakeTimeout.get());
        }

        /** Asılma — alliance'a göre pozisyon seçer (dashboard'dan ayarlanabilir) */
        private static Command autoClimb(ClimberSubsystem climber, DriveSubsystem drive) {
                return new AutoClimbCommand(
                                climber, drive,
                                () -> FieldConstants.getClimbPose(DriverStation.getAlliance()));
        }
}

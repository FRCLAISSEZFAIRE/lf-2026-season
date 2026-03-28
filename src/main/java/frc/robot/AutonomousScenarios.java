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
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.TunableNumber;

/**
 * Defines autonomous scenarios.
 * 
 * <p>
 * To add a new scenario:
 * 1. Add a new private static method below
 * 2. Register it in buildChooser() with addOption()
 * </p>
 * 
 * <p>
 * Timing settings are adjustable from the Dashboard:
 * Tuning/Auto/IntakeTimeout, StartShootTimeout, FullShootTimeout
 * </p>
 */
public final class AutonomousScenarios {

        // ==========================================================================
        // TIMING SETTINGS - Adjustable from Dashboard, saved to RIO
        // ==========================================================================
        private static final TunableNumber fullShootTimeout = new TunableNumber("Auto", "FullShootTimeout", 5.0);

        private static final SendableChooser<String> passChooser = new SendableChooser<>();

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

                passChooser.setDefaultOption("Trench", "Trench");
                passChooser.addOption("Bump", "Bump");
                SmartDashboard.putData("Pass/Pass Mode", passChooser);

                SmartDashboard.putData("Auto Chooser", chooser);

                return chooser;
        }

        // ==========================================================================
        // SCENARIO DEFINITIONS
        // ==========================================================================

        /**
         * Scenario 1: TrenchPass -> autoIntake -> TrenchPass -> Shoot
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
         * Scenario 2: Drive to Source (intake running) -> Shoot until auto ends
         */
        private static Command sourceAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // Drive to source while running intake + feeder
                                Commands.deadline(
                                                new SimpleDriveToPose(drive,
                                                                FieldConstants.getSourcePose(
                                                                                DriverStation.getAlliance())),
                                                Commands.runEnd(
                                                                () -> {
                                                                        intake.setExtensionPosition(IntakeConstants.kExtensionDeployedCm);
                                                                        intake.runRollerRPM(2000);
                                                                        feeder.feed();
                                                                },
                                                                () -> {
                                                                        intake.stopRoller();
                                                                        intake.setExtensionPosition(IntakeConstants.kExtensionRetractedCm);
                                                                        feeder.stop();
                                                                },
                                                                intake, feeder)),
                                // Shoot until auto ends
                                fullShoot(shooter, feeder, drive, intake));
        }

        /**
         * Scenario 3: TrenchPass -> autoIntake -> TrenchPass -> Shoot -> TrenchPass ->
         * autoIntake -> Shoot
         */
        private static Command doubleCollectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // Round 1
                                getMainPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                getMainPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive, intake),
                                // Round 2
                                getMainPassCommand(drive, shooter),
                                autoIntake(drive, intake, feeder),
                                fullShoot(shooter, feeder, drive, intake));
        }

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
                }, java.util.Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive, shooter));
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
                }, java.util.Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive, shooter));
        }

        // ==========================================================================
        // HELPER COMMANDS
        // ==========================================================================

        /** Full shoot - timeout adjustable from dashboard */
        private static Command fullShoot(
                        ShooterSubsystem shooter, FeederSubsystem feeder, DriveSubsystem drive,
                        IntakeSubsystem intake) {
                return new ShootCommand(shooter, feeder, drive, intake, drive::getPose)
                                .withTimeout(fullShootTimeout.get());
        }

        /**
         * Ball collection - drives to FeedStart/FeedStop points while running intake
         */
        private static Command autoIntake(
                        DriveSubsystem drive, IntakeSubsystem intake, FeederSubsystem feeder) {
                return Commands.deadline(
                                new FeedPassCommand(drive, intake),
                                Commands.runEnd(() -> feeder.feed(), () -> feeder.stop(), feeder));
        }

}

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
 * <p>Dashboard Choosers:</p>
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
                                // Drive to START point while collecting
                                autoIntakeToPoint(drive, intake, feeder, true),
                                // Pass back
                                getMainPassCommand(drive, shooter),
                                // Shoot
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
                                                // DeferredCommand: alliance is read at runtime, not build time
                                                new DeferredCommand(() -> new SimpleDriveToPose(drive,
                                                                FieldConstants.getSourcePose(
                                                                                DriverStation.getAlliance())),
                                                                Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(
                                                                                drive)),
                                                Commands.runEnd(
                                                                () -> {
                                                                        intake.setExtensionPosition(
                                                                                        IntakeConstants.kExtensionDeployedCm);
                                                                        intake.runRollerRPM(2000);
                                                                        feeder.feed();
                                                                },
                                                                () -> {
                                                                        intake.stopRoller();
                                                                        intake.setExtensionPosition(
                                                                                        IntakeConstants.kExtensionRetractedCm);
                                                                        feeder.stop();
                                                                },
                                                                intake, feeder)),
                                // Shoot until auto ends
                                fullShoot(shooter, feeder, drive, intake));
        }

        /**
         * Scenario 3: Pass -> autoIntake(to START) -> Pass back -> Shoot ->
         * Pass -> autoIntake(to STOP) -> Shoot
         */
        private static Command doubleCollectAndShoot(
                        DriveSubsystem drive, ShooterSubsystem shooter,
                        FeederSubsystem feeder, IntakeSubsystem intake) {

                return Commands.sequence(
                                // Round 1: intake to START point
                                getMainPassCommand(drive, shooter),
                                autoIntakeToPoint(drive, intake, feeder, true),
                                getMainPassCommand(drive, shooter),
                                fullShoot(shooter, feeder, drive, intake),
                                // Round 2: intake to STOP point
                                getMainPassCommand(drive, shooter),
                                autoIntakeToPoint(drive, intake, feeder, false),
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
         * Drives to a pass point while running intake + feeder.
         * @param isStart true = drive to START point (A/C), false = drive to STOP point (B/D)
         */
        private static Command autoIntakeToPoint(
                        DriveSubsystem drive, IntakeSubsystem intake,
                        FeederSubsystem feeder, boolean isStart) {
                return Commands.deadline(
                                // Drive to the target point
                                new DeferredCommand(() -> {
                                        Pose2d target = getPassPoint(isStart);
                                        System.out.println("[Auto] Intake driving to "
                                                        + (isStart ? "START" : "STOP") + ": " + target);
                                        return new SimpleDriveToPose(drive, target);
                                }, Set.<edu.wpi.first.wpilibj2.command.Subsystem>of(drive)),
                                // Run intake + feeder while driving
                                Commands.runEnd(
                                                () -> {
                                                        intake.setExtensionPosition(
                                                                        IntakeConstants.kExtensionDeployedCm);
                                                        intake.runRollerRPM(2000);
                                                        feeder.feed();
                                                },
                                                () -> {
                                                        intake.stopRoller();
                                                        intake.setExtensionPosition(
                                                                        IntakeConstants.kExtensionRetractedCm);
                                                        feeder.stop();
                                                },
                                                intake, feeder));
        }

}

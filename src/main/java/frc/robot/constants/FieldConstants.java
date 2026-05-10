package frc.robot.constants;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.TunableNumber;

/**
 * Field Constants for FRC 2026 Rebuilt Game.
 * Loads field layout from "2026-rebuilt-welded.json" and calculates centroids.
 * UPDATED: Uses TunableNumber for critical field elements to allow live
 * adjustment.
 */
public final class FieldConstants {

        // =====================================================================
        // UNITS: METERS
        // All coordinates and dimensions are in METERS unless otherwise specified.
        // =====================================================================

        public static final double kFieldLengthMeters = 16.49;
        public static final double kFieldWidthMeters = 8.10;

        // Layout
        public static AprilTagFieldLayout fieldLayout;

        // Tag Lists
        private static final List<Integer> RED_HANG_TAGS = List.of(15, 16);
        private static final List<Integer> BLUE_HANG_TAGS = List.of(31, 32);

        // Calculated Centers (Cached)
        // Removed static final Translation2d for Hub, Pass, Feeding to allow tuning.
        // Hang centers still depend on AprilTags or defaults.
        public static final Translation2d RED_HANG_CENTER;
        public static final Translation2d BLUE_HANG_CENTER;

        // === TUNABLE FIELD TARGETS ===
        // Group: "Field"

        // HUB
        public static final Translation2d BLUE_HUB_CENTER = new Translation2d(4.6, 4.0);
        public static final Translation2d RED_HUB_CENTER = new Translation2d(12.0, 4.0);

        // FEEDING (Source/Station Target)
        private static final double blueFeedingX = 3.5;
        private static final double blueFeedingY = 4.05;

        private static final double redFeedingX = 12.99;
        private static final double redFeedingY = 4.05;

        // PASS TARGETS (Left/Right)
        // Blue Pass
        private static final double bluePassRightX = 1.2;
        private static final double bluePassRightY = 1.0;

        private static final double bluePassLeftX = 1.2;
        private static final double bluePassLeftY = 6.5;

        // Red Pass
        private static final double redPassRightX = 15.6;
        private static final double redPassRightY = 6.6;

        private static final double redPassLeftX = 15.6;
        private static final double redPassLeftY = 1.0;

        // TOWER
        private static final double towerX = 8.245;
        private static final double towerY = 4.05;

        // SOURCE / DEPOT POSITIONS (Per Alliance)
        private static final double depotBlueX = 1.1;
        private static final double depotBlueY = 5.8;
        private static final double depotBlueAngle = 180.0;

        private static final double depotRedX = 15.4;
        private static final double depotRedY = 2.0;
        private static final double depotRedAngle = 0.0;

        // TRENCH PASS POINTS (Blue Alliance has A, B, C, D)
        private static final double trenchBlueA_X = 3.270;
        private static final double trenchBlueA_Y = 0.650;
        public static final double trenchBlueA_Angle = 0.0;

        private static final double trenchBlueB_X = 5.700;
        private static final double trenchBlueB_Y = 0.650;
        public static final double trenchBlueB_Angle = 0.0;

        private static final double trenchBlueC_X = 3.270;
        private static final double trenchBlueC_Y = 7.410;
        public static final double trenchBlueC_Angle = 0.0;

        private static final double trenchBlueD_X = 5.700;
        private static final double trenchBlueD_Y = 7.410;
        public static final double trenchBlueD_Angle = 0.0;

        // TRENCH PASS POINTS (Red Alliance has A, B, C, D)
        private static final double trenchRedA_X = 13.400;
        private static final double trenchRedA_Y = 7.470;
        public static final double trenchRedA_Angle = 180.0;

        private static final double trenchRedB_X = 10.800;
        private static final double trenchRedB_Y = 7.470;
        public static final double trenchRedB_Angle = 180.0;

        private static final double trenchRedC_X = 13.400;
        private static final double trenchRedC_Y = 0.650;
        public static final double trenchRedC_Angle = 180.0;

        private static final double trenchRedD_X = 10.800;
        private static final double trenchRedD_Y = 0.650;
        public static final double trenchRedD_Angle = 180.0;

        // BUMP PASS POINTS (Blue Alliance has A, B, C, D)
        private static final double bumpBlueA_X = 3.270;
        private static final double bumpBlueA_Y = 2.7;
        private static final double bumpBlueA_Angle = 0.0;

        private static final double bumpBlueB_X = 5.9;
        private static final double bumpBlueB_Y = 2.3;
        private static final double bumpBlueB_Angle = 0.0;

        private static final double bumpBlueC_X = 3.270;
        private static final double bumpBlueC_Y = 5.5;
        private static final double bumpBlueC_Angle = 0.0;

        private static final double bumpBlueD_X = 5.9;
        private static final double bumpBlueD_Y = 5.5;
        private static final double bumpBlueD_Angle = 0.0;

        // BUMP PASS POINTS (Red Alliance has A, B, C, D)
        private static final double bumpRedA_X = 13.3;
        private static final double bumpRedA_Y = 5.58;
        private static final double bumpRedA_Angle = 180.0;

        private static final double bumpRedB_X = 10.4;
        private static final double bumpRedB_Y = 5.58;
        private static final double bumpRedB_Angle = 180.0;

        private static final double bumpRedC_X = 13.3;
        private static final double bumpRedC_Y = 2.42;
        private static final double bumpRedC_Angle = 180.0;

        private static final double bumpRedD_X = 10.4;
        private static final double bumpRedD_Y = 2.42;
        private static final double bumpRedD_Angle = 180.0;

        // OUTPOST POSITIONS (Per Alliance)
        private static final double outpostBlueX = 0.4;
        private static final double outpostBlueY = 0.6;
        private static final double outpostBlueAngle = 90.0;

        private static final double outpostRedX = 16.1;
        private static final double outpostRedY = 7.4;
        private static final double outpostRedAngle = -90.0;

        // =====================================================================
        // FEED PASS POSITIONS (Auto Collection Paths)
        // =====================================================================

        // --- BLUE ALLIANCE FEED PASS (Diagonal Paths for Maximum Collection) ---
        // Left Path (Y High to Low Diagonal)
        private static final double feedStartBlueLeftX = 7.3;
        private static final double feedStartBlueLeftY = 6;
        public static final double feedStartBlueLeftAngle = -70;

        private static final double feedStopBlueLeftX = 7.3;
        private static final double feedStopBlueLeftY = 5.8;
        public static final double feedStopBlueLeftAngle = -70;

        // Right Path (Y Low to High Diagonal)
        public static final double feedStartBlueRightX = 7.6;
        public static final double feedStartBlueRightY = 1.8;
        public static final double feedStartBlueRightAngle = 70;

        private static final double feedStopBlueRightX = 7.6;
        private static final double feedStopBlueRightY = 1.8;
        public static final double feedStopBlueRightAngle = 70;

        // --- RED ALLIANCE FEED PASS (Diagonal Paths for Maximum Collection) ---
        // Left Path (Y High to Low Diagonal)
        private static final double feedStartRedLeftX = 9;
        private static final double feedStartRedLeftY = 2.5;
        public static final double feedStartRedLeftAngle = 140.0;

        private static final double feedStopRedLeftX = 9;
        private static final double feedStopRedLeftY = 2.7;
        public static final double feedStopRedLeftAngle = 140.0;

        // Right Path (Y Low to High Diagonal)
        private static final double feedStartRedRightX = 9;
        private static final double feedStartRedRightY = 5.7;
        public static final double feedStartRedRightAngle = -130;

        private static final double feedStopRedRightX = 9;
        private static final double feedStopRedRightY = 5.5;
        public static final double feedStopRedRightAngle = -130;

        // === LEGACY CONSTANTS (Restored & Linked) ===
        public static final double kHubHeightMeters = 1.83;

        // Poses that depend on Tunables - WE CANNOT MAKE THEM CONSTANTS ANYMORE
        // We will provide methods to get them dynamically.

        public static final Pose2d kBlueStartPose = new Pose2d(2.0, 4.05, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedStartPose = new Pose2d(14.49, 4.05, Rotation2d.fromDegrees(180));
        public static final Pose2d kTestStartPose = new Pose2d(8.245, 4.05, Rotation2d.fromDegrees(0));

        public static final double kBlueAllianceZoneMaxX = 4.0;
        public static final double kRedAllianceZoneMinX = 12.49;

        public static final double kIdealShootingDistanceMeters = 3.0;

        public static final Pose2d kBlueOutpostPose = new Pose2d(15.0, 7.0, Rotation2d.fromDegrees(180));
        public static final Pose2d kRedOutpostPose = new Pose2d(1.49, 7.0, Rotation2d.fromDegrees(0));

        public static final double kFuelDiameterMeters = 0.15;

        public static final List<Translation2d> kKeepOutZones = new ArrayList<>();

        static {
                Translation2d redHang = new Translation2d();
                Translation2d blueHang = new Translation2d();

                try {
                        // Load Layout
                        String filename = "2026-rebuilt-welded.json";
                        Path path = Filesystem.getDeployDirectory().toPath().resolve(filename);
                        fieldLayout = new AprilTagFieldLayout(path);

                        // Calculate Hang Centers (Static)
                        redHang = calculateAverageCentroid(RED_HANG_TAGS);
                        blueHang = calculateAverageCentroid(BLUE_HANG_TAGS);

                        System.out.println(
                                        "[FieldConstants] Layout Loaded. Hang Centers Calc'd.");

                } catch (IOException e) {
                        System.err.println("[FieldConstants] ERROR: Could not load " + "2026-rebuilt-welded.json");
                        e.printStackTrace();
                        redHang = new Translation2d(11.0, 4.05); // Approximate
                        blueHang = new Translation2d(5.0, 4.05); // Approximate
                }

                RED_HANG_CENTER = redHang;
                BLUE_HANG_CENTER = blueHang;
        }

        /**
         * Calculates the average Translation2d of a list of AprilTags.
         */
        private static Translation2d calculateAverageCentroid(List<Integer> tagIds) {
                double xSum = 0;
                double ySum = 0;
                int count = 0;

                for (int id : tagIds) {
                        Optional<Pose3d> tagPose = fieldLayout.getTagPose(id);
                        if (tagPose.isPresent()) {
                                xSum += tagPose.get().getX();
                                ySum += tagPose.get().getY();
                                count++;
                        }
                }

                if (count == 0)
                        return new Translation2d();
                return new Translation2d(xSum / count, ySum / count);
        }

        // =====================================================================
        // DYNAMIC GETTERS (TUNABLE)
        // =====================================================================

        /**
         * Gets the Hub Center for the specified alliance.
         */
        public static Translation2d getHubCenter(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return RED_HUB_CENTER;
                }
                return BLUE_HUB_CENTER;
        }

        /**
         * Gets the Hang Center for the specified alliance.
         */
        public static Translation2d getHangCenter(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return RED_HANG_CENTER;
                }
                return BLUE_HANG_CENTER;
        }

        /**
         * Gets Trench Pass Point A for Blue Alliance
         */
        public static Pose2d getTrenchPointBlueA() {
                return new Pose2d(trenchBlueA_X, trenchBlueA_Y,
                                Rotation2d.fromDegrees(trenchBlueA_Angle));
        }

        public static Pose2d getTrenchPointBlueB() {
                return new Pose2d(trenchBlueB_X, trenchBlueB_Y,
                                Rotation2d.fromDegrees(trenchBlueB_Angle));
        }

        public static Pose2d getTrenchPointBlueC() {
                return new Pose2d(trenchBlueC_X, trenchBlueC_Y,
                                Rotation2d.fromDegrees(trenchBlueC_Angle));
        }

        public static Pose2d getTrenchPointBlueD() {
                return new Pose2d(trenchBlueD_X, trenchBlueD_Y,
                                Rotation2d.fromDegrees(trenchBlueD_Angle));
        }

        public static Pose2d getTrenchPointRedA() {
                return new Pose2d(trenchRedA_X, trenchRedA_Y,
                                Rotation2d.fromDegrees(trenchRedA_Angle));
        }

        public static Pose2d getTrenchPointRedB() {
                return new Pose2d(trenchRedB_X, trenchRedB_Y,
                                Rotation2d.fromDegrees(trenchRedB_Angle));
        }

        public static Pose2d getTrenchPointRedC() {
                return new Pose2d(trenchRedC_X, trenchRedC_Y,
                                Rotation2d.fromDegrees(trenchRedC_Angle));
        }

        public static Pose2d getTrenchPointRedD() {
                return new Pose2d(trenchRedD_X, trenchRedD_Y,
                                Rotation2d.fromDegrees(trenchRedD_Angle));
        }

        // --- BUMP PASS POINT GETTERS ---

        public static Pose2d getBumpPointBlueA() {
                return new Pose2d(bumpBlueA_X, bumpBlueA_Y,
                                Rotation2d.fromDegrees(bumpBlueA_Angle));
        }

        public static Pose2d getBumpPointBlueB() {
                return new Pose2d(bumpBlueB_X, bumpBlueB_Y,
                                Rotation2d.fromDegrees(bumpBlueB_Angle));
        }

        public static Pose2d getBumpPointBlueC() {
                return new Pose2d(bumpBlueC_X, bumpBlueC_Y,
                                Rotation2d.fromDegrees(bumpBlueC_Angle));
        }

        public static Pose2d getBumpPointBlueD() {
                return new Pose2d(bumpBlueD_X, bumpBlueD_Y,
                                Rotation2d.fromDegrees(bumpBlueD_Angle));
        }

        public static Pose2d getBumpPointRedA() {
                return new Pose2d(bumpRedA_X, bumpRedA_Y,
                                Rotation2d.fromDegrees(bumpRedA_Angle));
        }

        public static Pose2d getBumpPointRedB() {
                return new Pose2d(bumpRedB_X, bumpRedB_Y,
                                Rotation2d.fromDegrees(bumpRedB_Angle));
        }

        public static Pose2d getBumpPointRedC() {
                return new Pose2d(bumpRedC_X, bumpRedC_Y,
                                Rotation2d.fromDegrees(bumpRedC_Angle));
        }

        public static Pose2d getBumpPointRedD() {
                return new Pose2d(bumpRedD_X, bumpRedD_Y,
                                Rotation2d.fromDegrees(bumpRedD_Angle));
        }

        // --- FEED PASS TARGET GETTERS ---

        public static Pose2d getFeedStartBlueLeft() {
                return new Pose2d(feedStartBlueLeftX, feedStartBlueLeftY,
                                Rotation2d.fromDegrees(feedStartBlueLeftAngle));
        }

        public static Pose2d getFeedStopBlueLeft() {
                return new Pose2d(feedStopBlueLeftX, feedStopBlueLeftY,
                                Rotation2d.fromDegrees(feedStopBlueLeftAngle));
        }

        public static Pose2d getFeedStartBlueRight() {
                return new Pose2d(feedStartBlueRightX, feedStartBlueRightY,
                                Rotation2d.fromDegrees(feedStartBlueRightAngle));
        }

        public static Pose2d getFeedStopBlueRight() {
                return new Pose2d(feedStopBlueRightX, feedStopBlueRightY,
                                Rotation2d.fromDegrees(feedStopBlueRightAngle));
        }

        public static Pose2d getFeedStartRedLeft() {
                return new Pose2d(feedStartRedLeftX, feedStartRedLeftY,
                                Rotation2d.fromDegrees(feedStartRedLeftAngle));
        }

        public static Pose2d getFeedStopRedLeft() {
                return new Pose2d(feedStopRedLeftX, feedStopRedLeftY,
                                Rotation2d.fromDegrees(feedStopRedLeftAngle));
        }

        public static Pose2d getFeedStartRedRight() {
                return new Pose2d(feedStartRedRightX, feedStartRedRightY,
                                Rotation2d.fromDegrees(feedStartRedRightAngle));
        }

        public static Pose2d getFeedStopRedRight() {
                return new Pose2d(feedStopRedRightX, feedStopRedRightY,
                                Rotation2d.fromDegrees(feedStopRedRightAngle));
        }

        /**
         * Gets the Pass Target (Feeding Station area) for the specified alliance.
         * Selects the closest target (Left/Right) based on Robot's Y position.
         */
        public static Translation2d getPassTarget(Optional<Alliance> alliance, Pose2d robotPose) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        // RED ALLIANCE
                        // Compare Y distance to Left and Right targets
                        double rRightY = redPassRightY;
                        double rLeftY = redPassLeftY;

                        double distRight = Math.abs(robotPose.getY() - rRightY);
                        double distLeft = Math.abs(robotPose.getY() - rLeftY);

                        if (distLeft < distRight) {
                                return new Translation2d(redPassLeftX, rLeftY);
                        } else {
                                return new Translation2d(redPassRightX, rRightY);
                        }
                } else {
                        // BLUE ALLIANCE (Default)
                        double bRightY = bluePassRightY;
                        double bLeftY = bluePassLeftY;

                        double distRight = Math.abs(robotPose.getY() - bRightY);
                        double distLeft = Math.abs(robotPose.getY() - bLeftY);

                        if (distLeft < distRight) {
                                return new Translation2d(bluePassLeftX, bLeftY);
                        } else {
                                return new Translation2d(bluePassRightX, bRightY);
                        }
                }
        }

        /**
         * Gets the Feeding Target (Source/Station) for the specified alliance.
         */
        public static Translation2d getFeedingTarget(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return new Translation2d(redFeedingX, redFeedingY);
                }
                return new Translation2d(blueFeedingX, blueFeedingY);
        }

        public static Translation2d getTowerCenter() {
                return new Translation2d(towerX, towerY);
        }

        /**
         * Gets the Outpost Pose for the specified alliance.
         */
        public static Pose2d getOutpostPose(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return new Pose2d(outpostRedX, outpostRedY,
                                        Rotation2d.fromDegrees(outpostRedAngle));
                }
                return new Pose2d(outpostBlueX, outpostBlueY,
                                Rotation2d.fromDegrees(outpostBlueAngle));
        }

        /**
         * Gets the Source (Depot/Station) pose for the specified alliance.
         * Tunable from Dashboard: Field/DepotRed/*, Field/DepotBlue/*
         */
        public static Pose2d getSourcePose(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return new Pose2d(depotRedX, depotRedY,
                                        Rotation2d.fromDegrees(depotRedAngle));
                }
                return new Pose2d(depotBlueX, depotBlueY,
                                Rotation2d.fromDegrees(depotBlueAngle));
        }

        private FieldConstants() {
        }
}

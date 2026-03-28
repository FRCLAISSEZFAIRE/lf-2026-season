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
        private static final TunableNumber blueHubX = new TunableNumber("Field", "BlueHub/X", 4.6);
        private static final TunableNumber blueHubY = new TunableNumber("Field", "BlueHub/Y", 4.0);

        private static final TunableNumber redHubX = new TunableNumber("Field", "RedHub/X", 12);
        private static final TunableNumber redHubY = new TunableNumber("Field", "RedHub/Y", 4);

        // FEEDING (Source/Station Target)
        private static final TunableNumber blueFeedingX = new TunableNumber("Field", "BlueFeeding/X", 3.5);
        private static final TunableNumber blueFeedingY = new TunableNumber("Field", "BlueFeeding/Y", 4.05);

        private static final TunableNumber redFeedingX = new TunableNumber("Field", "RedFeeding/X", 12.99);
        private static final TunableNumber redFeedingY = new TunableNumber("Field", "RedFeeding/Y", 4.05);

        // PASS TARGETS (Left/Right)
        // Blue Pass
        private static final TunableNumber bluePassRightX = new TunableNumber("Field", "BluePassRight/X", 1.2);
        private static final TunableNumber bluePassRightY = new TunableNumber("Field", "BluePassRight/Y", 1.0);

        private static final TunableNumber bluePassLeftX = new TunableNumber("Field", "BluePassLeft/X", 1.2);
        private static final TunableNumber bluePassLeftY = new TunableNumber("Field", "BluePassLeft/Y", 6.5);

        // Red Pass
        private static final TunableNumber redPassRightX = new TunableNumber("Field", "RedPassRight/X", 15.6);
        private static final TunableNumber redPassRightY = new TunableNumber("Field", "RedPassRight/Y", 6.6);

        private static final TunableNumber redPassLeftX = new TunableNumber("Field", "RedPassLeft/X", 15.6);
        private static final TunableNumber redPassLeftY = new TunableNumber("Field", "RedPassLeft/Y", 1.0);

        // TOWER
        private static final TunableNumber towerX = new TunableNumber("Field", "Tower/X", 8.245);
        private static final TunableNumber towerY = new TunableNumber("Field", "Tower/Y", 4.05);

        // CLIMB POSITIONS (Per Alliance)
        private static final TunableNumber climbBlueX = new TunableNumber("Field", "ClimbBlue/X", 1.44);
        private static final TunableNumber climbBlueY = new TunableNumber("Field", "ClimbBlue/Y", 3.75);
        private static final TunableNumber climbBlueAngle = new TunableNumber("Field", "ClimbBlue/Angle", -90.0);

        private static final TunableNumber climbRedX = new TunableNumber("Field", "ClimbRed/X", 15.16);
        private static final TunableNumber climbRedY = new TunableNumber("Field", "ClimbRed/Y", 4.34);
        private static final TunableNumber climbRedAngle = new TunableNumber("Field", "ClimbRed/Angle", 270.0);

        // TRENCH PASS POINTS (Blue Alliance has A, B, C, D)
        private static final TunableNumber trenchBlueA_X = new TunableNumber("Field", "TrenchBlueA/X", 3.270);
        private static final TunableNumber trenchBlueA_Y = new TunableNumber("Field", "TrenchBlueA/Y", 0.650);
        private static final TunableNumber trenchBlueA_Angle = new TunableNumber("Field", "TrenchBlueA/Angle",
                        0.0);

        private static final TunableNumber trenchBlueB_X = new TunableNumber("Field", "TrenchBlueB/X", 5.700);
        private static final TunableNumber trenchBlueB_Y = new TunableNumber("Field", "TrenchBlueB/Y", 0.650);
        private static final TunableNumber trenchBlueB_Angle = new TunableNumber("Field", "TrenchBlueB/Angle",
                        0.0);

        private static final TunableNumber trenchBlueC_X = new TunableNumber("Field", "TrenchBlueC/X", 3.270);
        private static final TunableNumber trenchBlueC_Y = new TunableNumber("Field", "TrenchBlueC/Y", 7.410);
        private static final TunableNumber trenchBlueC_Angle = new TunableNumber("Field", "TrenchBlueC/Angle",
                        0.0);

        private static final TunableNumber trenchBlueD_X = new TunableNumber("Field", "TrenchBlueD/X", 5.700);
        private static final TunableNumber trenchBlueD_Y = new TunableNumber("Field", "TrenchBlueD/Y", 7.410);
        private static final TunableNumber trenchBlueD_Angle = new TunableNumber("Field", "TrenchBlueD/Angle",
                        0.0);

        // TRENCH PASS POINTS (Red Alliance has A, B, C, D)
        private static final TunableNumber trenchRedA_X = new TunableNumber("Field", "TrenchRedA/X", 13.400);
        private static final TunableNumber trenchRedA_Y = new TunableNumber("Field", "TrenchRedA/Y", 7.470);
        private static final TunableNumber trenchRedA_Angle = new TunableNumber("Field", "TrenchRedA/Angle",
                        180.0);

        private static final TunableNumber trenchRedB_X = new TunableNumber("Field", "TrenchRedB/X", 10.800);
        private static final TunableNumber trenchRedB_Y = new TunableNumber("Field", "TrenchRedB/Y", 7.470);
        private static final TunableNumber trenchRedB_Angle = new TunableNumber("Field", "TrenchRedB/Angle",
                        180.0);

        private static final TunableNumber trenchRedC_X = new TunableNumber("Field", "TrenchRedC/X", 13.400);
        private static final TunableNumber trenchRedC_Y = new TunableNumber("Field", "TrenchRedC/Y", 0.650);
        private static final TunableNumber trenchRedC_Angle = new TunableNumber("Field", "TrenchRedC/Angle",
                        180.0);

        private static final TunableNumber trenchRedD_X = new TunableNumber("Field", "TrenchRedD/X", 10.800);
        private static final TunableNumber trenchRedD_Y = new TunableNumber("Field", "TrenchRedD/Y", 0.650);
        private static final TunableNumber trenchRedD_Angle = new TunableNumber("Field", "TrenchRedD/Angle",
                        180.0);

        // BUMP PASS POINTS (Blue Alliance has A, B, C, D)
        private static final TunableNumber bumpBlueA_X = new TunableNumber("Field", "BumpBlueA/X", 3.270);
        private static final TunableNumber bumpBlueA_Y = new TunableNumber("Field", "BumpBlueA/Y", 2.7);
        private static final TunableNumber bumpBlueA_Angle = new TunableNumber("Field", "BumpBlueA/Angle", 0.0);

        private static final TunableNumber bumpBlueB_X = new TunableNumber("Field", "BumpBlueB/X", 5.9);
        private static final TunableNumber bumpBlueB_Y = new TunableNumber("Field", "BumpBlueB/Y", 2.3);
        private static final TunableNumber bumpBlueB_Angle = new TunableNumber("Field", "BumpBlueB/Angle", 0.0);

        private static final TunableNumber bumpBlueC_X = new TunableNumber("Field", "BumpBlueC/X", 3.270);
        private static final TunableNumber bumpBlueC_Y = new TunableNumber("Field", "BumpBlueC/Y", 5.5);
        private static final TunableNumber bumpBlueC_Angle = new TunableNumber("Field", "BumpBlueC/Angle", 0.0);

        private static final TunableNumber bumpBlueD_X = new TunableNumber("Field", "BumpBlueD/X", 5.9);
        private static final TunableNumber bumpBlueD_Y = new TunableNumber("Field", "BumpBlueD/Y", 5.5);
        private static final TunableNumber bumpBlueD_Angle = new TunableNumber("Field", "BumpBlueD/Angle", 0.0);

        // BUMP PASS POINTS (Red Alliance has A, B, C, D)
        private static final TunableNumber bumpRedA_X = new TunableNumber("Field", "BumpRedA/X", 13.3);
        private static final TunableNumber bumpRedA_Y = new TunableNumber("Field", "BumpRedA/Y", 5.58);
        private static final TunableNumber bumpRedA_Angle = new TunableNumber("Field", "BumpRedA/Angle", 180.0);

        private static final TunableNumber bumpRedB_X = new TunableNumber("Field", "BumpRedB/X", 10.4);
        private static final TunableNumber bumpRedB_Y = new TunableNumber("Field", "BumpRedB/Y", 5.58);
        private static final TunableNumber bumpRedB_Angle = new TunableNumber("Field", "BumpRedB/Angle", 180.0);

        private static final TunableNumber bumpRedC_X = new TunableNumber("Field", "BumpRedC/X", 13.3);
        private static final TunableNumber bumpRedC_Y = new TunableNumber("Field", "BumpRedC/Y", 2.42);
        private static final TunableNumber bumpRedC_Angle = new TunableNumber("Field", "BumpRedC/Angle", 180.0);

        private static final TunableNumber bumpRedD_X = new TunableNumber("Field", "BumpRedD/X", 10.4);
        private static final TunableNumber bumpRedD_Y = new TunableNumber("Field", "BumpRedD/Y", 2.42);
        private static final TunableNumber bumpRedD_Angle = new TunableNumber("Field", "BumpRedD/Angle", 180.0);

        // OUTPOST POSITIONS (Per Alliance)
        private static final TunableNumber outpostBlueX = new TunableNumber("Field", "OutpostBlue/X", 0.4);
        private static final TunableNumber outpostBlueY = new TunableNumber("Field", "OutpostBlue/Y", 0.6);
        private static final TunableNumber outpostBlueAngle = new TunableNumber("Field", "OutpostBlue/Angle", 90.0);

        private static final TunableNumber outpostRedX = new TunableNumber("Field", "OutpostRed/X", 16.1);
        private static final TunableNumber outpostRedY = new TunableNumber("Field", "OutpostRed/Y", 7.4);
        private static final TunableNumber outpostRedAngle = new TunableNumber("Field", "OutpostRed/Angle", -90.0);

        // =====================================================================
        // FEED PASS POSITIONS (Auto Collection Paths)
        // =====================================================================

        // --- BLUE ALLIANCE FEED PASS (Diagonal Paths for Maximum Collection) ---
        // Left Path (Y High to Low Diagonal)
        private static final TunableNumber feedStartBlueLeftX = new TunableNumber("Field", "FeedStartBlueLeft/X", 7.3);
        private static final TunableNumber feedStartBlueLeftY = new TunableNumber("Field", "FeedStartBlueLeft/Y", 6);
        private static final TunableNumber feedStartBlueLeftAngle = new TunableNumber("Field",
                        "FeedStartBlueLeft/Angle", -70);

        private static final TunableNumber feedStopBlueLeftX = new TunableNumber("Field", "FeedStopBlueLeft/X", 7.3);
        private static final TunableNumber feedStopBlueLeftY = new TunableNumber("Field", "FeedStopBlueLeft/Y", 5.8);
        private static final TunableNumber feedStopBlueLeftAngle = new TunableNumber("Field", "FeedStopBlueLeft/Angle",
                        -70);

        // Right Path (Y Low to High Diagonal)
        private static final TunableNumber feedStartBlueRightX = new TunableNumber("Field", "FeedStartBlueRight/X",
                        7.6);
        private static final TunableNumber feedStartBlueRightY = new TunableNumber("Field", "FeedStartBlueRight/Y",
                        1.8);
        private static final TunableNumber feedStartBlueRightAngle = new TunableNumber("Field",
                        "FeedStartBlueRight/Angle", 70);

        private static final TunableNumber feedStopBlueRightX = new TunableNumber("Field", "FeedStopBlueRight/X", 7.6);
        private static final TunableNumber feedStopBlueRightY = new TunableNumber("Field", "FeedStopBlueRight/Y", 1.8);
        private static final TunableNumber feedStopBlueRightAngle = new TunableNumber("Field",
                        "FeedStopBlueRight/Angle", 70);

        // --- RED ALLIANCE FEED PASS (Diagonal Paths for Maximum Collection) ---
        // Left Path (Y High to Low Diagonal)
        private static final TunableNumber feedStartRedLeftX = new TunableNumber("Field", "FeedStartRedLeft/X", 9);
        private static final TunableNumber feedStartRedLeftY = new TunableNumber("Field", "FeedStartRedLeft/Y", 2.5);
        private static final TunableNumber feedStartRedLeftAngle = new TunableNumber("Field", "FeedStartRedLeft/Angle",
                        140.0);

        private static final TunableNumber feedStopRedLeftX = new TunableNumber("Field", "FeedStopRedLeft/X", 9);
        private static final TunableNumber feedStopRedLeftY = new TunableNumber("Field", "FeedStopRedLeft/Y", 2.7);
        private static final TunableNumber feedStopRedLeftAngle = new TunableNumber("Field", "FeedStopRedLeft/Angle",
                        140.0);

        // Right Path (Y Low to High Diagonal)
        private static final TunableNumber feedStartRedRightX = new TunableNumber("Field", "FeedStartRedRight/X", 9);
        private static final TunableNumber feedStartRedRightY = new TunableNumber("Field", "FeedStartRedRight/Y", 5.7);
        private static final TunableNumber feedStartRedRightAngle = new TunableNumber("Field",
                        "FeedStartRedRight/Angle", -130);

        private static final TunableNumber feedStopRedRightX = new TunableNumber("Field", "FeedStopRedRight/X", 9);
        private static final TunableNumber feedStopRedRightY = new TunableNumber("Field", "FeedStopRedRight/Y", 5.5);
        private static final TunableNumber feedStopRedRightAngle = new TunableNumber("Field", "FeedStopRedRight/Angle",
                        -130);

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

        public static final Pose2d kBlueDepotPose = new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedDepotPose = new Pose2d(14.99, 1.0, Rotation2d.fromDegrees(180));

        public static final Pose2d kBlueOutpostPose = new Pose2d(15.0, 7.0, Rotation2d.fromDegrees(180));
        public static final Pose2d kRedOutpostPose = new Pose2d(1.49, 7.0, Rotation2d.fromDegrees(0));

        public static final double kTowerClimbHeightMeters = 1.5;

        public static final Pose2d[] kSourcePoses = new Pose2d[] {
                        kBlueDepotPose,
                        kBlueOutpostPose
        };

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
                        return new Translation2d(redHubX.get(), redHubY.get());
                }
                return new Translation2d(blueHubX.get(), blueHubY.get());
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
                return new Pose2d(trenchBlueA_X.get(), trenchBlueA_Y.get(),
                                Rotation2d.fromDegrees(trenchBlueA_Angle.get()));
        }

        public static Pose2d getTrenchPointBlueB() {
                return new Pose2d(trenchBlueB_X.get(), trenchBlueB_Y.get(),
                                Rotation2d.fromDegrees(trenchBlueB_Angle.get()));
        }

        public static Pose2d getTrenchPointBlueC() {
                return new Pose2d(trenchBlueC_X.get(), trenchBlueC_Y.get(),
                                Rotation2d.fromDegrees(trenchBlueC_Angle.get()));
        }

        public static Pose2d getTrenchPointBlueD() {
                return new Pose2d(trenchBlueD_X.get(), trenchBlueD_Y.get(),
                                Rotation2d.fromDegrees(trenchBlueD_Angle.get()));
        }

        public static Pose2d getTrenchPointRedA() {
                return new Pose2d(trenchRedA_X.get(), trenchRedA_Y.get(),
                                Rotation2d.fromDegrees(trenchRedA_Angle.get()));
        }

        public static Pose2d getTrenchPointRedB() {
                return new Pose2d(trenchRedB_X.get(), trenchRedB_Y.get(),
                                Rotation2d.fromDegrees(trenchRedB_Angle.get()));
        }

        public static Pose2d getTrenchPointRedC() {
                return new Pose2d(trenchRedC_X.get(), trenchRedC_Y.get(),
                                Rotation2d.fromDegrees(trenchRedC_Angle.get()));
        }

        public static Pose2d getTrenchPointRedD() {
                return new Pose2d(trenchRedD_X.get(), trenchRedD_Y.get(),
                                Rotation2d.fromDegrees(trenchRedD_Angle.get()));
        }

        // --- BUMP PASS POINT GETTERS ---

        public static Pose2d getBumpPointBlueA() {
                return new Pose2d(bumpBlueA_X.get(), bumpBlueA_Y.get(),
                                Rotation2d.fromDegrees(bumpBlueA_Angle.get()));
        }

        public static Pose2d getBumpPointBlueB() {
                return new Pose2d(bumpBlueB_X.get(), bumpBlueB_Y.get(),
                                Rotation2d.fromDegrees(bumpBlueB_Angle.get()));
        }

        public static Pose2d getBumpPointBlueC() {
                return new Pose2d(bumpBlueC_X.get(), bumpBlueC_Y.get(),
                                Rotation2d.fromDegrees(bumpBlueC_Angle.get()));
        }

        public static Pose2d getBumpPointBlueD() {
                return new Pose2d(bumpBlueD_X.get(), bumpBlueD_Y.get(),
                                Rotation2d.fromDegrees(bumpBlueD_Angle.get()));
        }

        public static Pose2d getBumpPointRedA() {
                return new Pose2d(bumpRedA_X.get(), bumpRedA_Y.get(),
                                Rotation2d.fromDegrees(bumpRedA_Angle.get()));
        }

        public static Pose2d getBumpPointRedB() {
                return new Pose2d(bumpRedB_X.get(), bumpRedB_Y.get(),
                                Rotation2d.fromDegrees(bumpRedB_Angle.get()));
        }

        public static Pose2d getBumpPointRedC() {
                return new Pose2d(bumpRedC_X.get(), bumpRedC_Y.get(),
                                Rotation2d.fromDegrees(bumpRedC_Angle.get()));
        }

        public static Pose2d getBumpPointRedD() {
                return new Pose2d(bumpRedD_X.get(), bumpRedD_Y.get(),
                                Rotation2d.fromDegrees(bumpRedD_Angle.get()));
        }

        // --- FEED PASS TARGET GETTERS ---

        public static Pose2d getFeedStartBlueLeft() {
                return new Pose2d(feedStartBlueLeftX.get(), feedStartBlueLeftY.get(),
                                Rotation2d.fromDegrees(feedStartBlueLeftAngle.get()));
        }

        public static Pose2d getFeedStopBlueLeft() {
                return new Pose2d(feedStopBlueLeftX.get(), feedStopBlueLeftY.get(),
                                Rotation2d.fromDegrees(feedStopBlueLeftAngle.get()));
        }

        public static Pose2d getFeedStartBlueRight() {
                return new Pose2d(feedStartBlueRightX.get(), feedStartBlueRightY.get(),
                                Rotation2d.fromDegrees(feedStartBlueRightAngle.get()));
        }

        public static Pose2d getFeedStopBlueRight() {
                return new Pose2d(feedStopBlueRightX.get(), feedStopBlueRightY.get(),
                                Rotation2d.fromDegrees(feedStopBlueRightAngle.get()));
        }

        public static Pose2d getFeedStartRedLeft() {
                return new Pose2d(feedStartRedLeftX.get(), feedStartRedLeftY.get(),
                                Rotation2d.fromDegrees(feedStartRedLeftAngle.get()));
        }

        public static Pose2d getFeedStopRedLeft() {
                return new Pose2d(feedStopRedLeftX.get(), feedStopRedLeftY.get(),
                                Rotation2d.fromDegrees(feedStopRedLeftAngle.get()));
        }

        public static Pose2d getFeedStartRedRight() {
                return new Pose2d(feedStartRedRightX.get(), feedStartRedRightY.get(),
                                Rotation2d.fromDegrees(feedStartRedRightAngle.get()));
        }

        public static Pose2d getFeedStopRedRight() {
                return new Pose2d(feedStopRedRightX.get(), feedStopRedRightY.get(),
                                Rotation2d.fromDegrees(feedStopRedRightAngle.get()));
        }

        /**
         * Gets the Pass Target (Feeding Station area) for the specified alliance.
         * Selects the closest target (Left/Right) based on Robot's Y position.
         */
        public static Translation2d getPassTarget(Optional<Alliance> alliance, Pose2d robotPose) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        // RED ALLIANCE
                        // Compare Y distance to Left and Right targets
                        double rRightY = redPassRightY.get();
                        double rLeftY = redPassLeftY.get();

                        double distRight = Math.abs(robotPose.getY() - rRightY);
                        double distLeft = Math.abs(robotPose.getY() - rLeftY);

                        if (distLeft < distRight) {
                                return new Translation2d(redPassLeftX.get(), rLeftY);
                        } else {
                                return new Translation2d(redPassRightX.get(), rRightY);
                        }
                } else {
                        // BLUE ALLIANCE (Default)
                        double bRightY = bluePassRightY.get();
                        double bLeftY = bluePassLeftY.get();

                        double distRight = Math.abs(robotPose.getY() - bRightY);
                        double distLeft = Math.abs(robotPose.getY() - bLeftY);

                        if (distLeft < distRight) {
                                return new Translation2d(bluePassLeftX.get(), bLeftY);
                        } else {
                                return new Translation2d(bluePassRightX.get(), bRightY);
                        }
                }
        }

        /**
         * Gets the Feeding Target (Source/Station) for the specified alliance.
         */
        public static Translation2d getFeedingTarget(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return new Translation2d(redFeedingX.get(), redFeedingY.get());
                }
                return new Translation2d(blueFeedingX.get(), blueFeedingY.get());
        }

        public static Translation2d getTowerCenter() {
                return new Translation2d(towerX.get(), towerY.get());
        }

        /**
         * Alliance'a göre asılma pozisyonunu döndürür.
         */
        public static Pose2d getClimbPose(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return new Pose2d(climbRedX.get(), climbRedY.get(),
                                        Rotation2d.fromDegrees(climbRedAngle.get()));
                }
                return new Pose2d(climbBlueX.get(), climbBlueY.get(),
                                Rotation2d.fromDegrees(climbBlueAngle.get()));
        }

        /**
         * Gets the Outpost Pose for the specified alliance.
         */
        public static Pose2d getOutpostPose(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return new Pose2d(outpostRedX.get(), outpostRedY.get(),
                                        Rotation2d.fromDegrees(outpostRedAngle.get()));
                }
                return new Pose2d(outpostBlueX.get(), outpostBlueY.get(),
                                Rotation2d.fromDegrees(outpostBlueAngle.get()));
        }

        private FieldConstants() {
        }
}

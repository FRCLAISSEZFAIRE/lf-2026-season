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
        private static final TunableNumber blueHubX = new TunableNumber("Field", "BlueHub/X", 4.05);
        private static final TunableNumber blueHubY = new TunableNumber("Field", "BlueHub/Y", 4.05);

        private static final TunableNumber redHubX = new TunableNumber("Field", "RedHub/X", 12.44);
        private static final TunableNumber redHubY = new TunableNumber("Field", "RedHub/Y", 4.05);

        // FEEDING (Source/Station Target)
        private static final TunableNumber blueFeedingX = new TunableNumber("Field", "BlueFeeding/X", 3.5);
        private static final TunableNumber blueFeedingY = new TunableNumber("Field", "BlueFeeding/Y", 4.05);

        private static final TunableNumber redFeedingX = new TunableNumber("Field", "RedFeeding/X", 12.99);
        private static final TunableNumber redFeedingY = new TunableNumber("Field", "RedFeeding/Y", 4.05);

        // PASS TARGETS (Left/Right)
        // Blue Pass
        private static final TunableNumber bluePassRightX = new TunableNumber("Field", "BluePassRight/X", 2.0);
        private static final TunableNumber bluePassRightY = new TunableNumber("Field", "BluePassRight/Y", 1.0);

        private static final TunableNumber bluePassLeftX = new TunableNumber("Field", "BluePassLeft/X", 2.0);
        private static final TunableNumber bluePassLeftY = new TunableNumber("Field", "BluePassLeft/Y", 7.0);

        // Red Pass
        private static final TunableNumber redPassRightX = new TunableNumber("Field", "RedPassRight/X", 14.5);
        private static final TunableNumber redPassRightY = new TunableNumber("Field", "RedPassRight/Y", 1.0);

        private static final TunableNumber redPassLeftX = new TunableNumber("Field", "RedPassLeft/X", 14.5);
        private static final TunableNumber redPassLeftY = new TunableNumber("Field", "RedPassLeft/Y", 7.0);

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

        // TRANSITION POINTS (Blue Alliance has A, B, C, D)
        private static final TunableNumber transitionBlueA_X = new TunableNumber("Field", "TransitionBlueA/X", 3.270);
        private static final TunableNumber transitionBlueA_Y = new TunableNumber("Field", "TransitionBlueA/Y", 0.650);
        private static final TunableNumber transitionBlueA_Angle = new TunableNumber("Field", "TransitionBlueA/Angle",
                        0.0);

        private static final TunableNumber transitionBlueB_X = new TunableNumber("Field", "TransitionBlueB/X", 5.700);
        private static final TunableNumber transitionBlueB_Y = new TunableNumber("Field", "TransitionBlueB/Y", 0.650);
        private static final TunableNumber transitionBlueB_Angle = new TunableNumber("Field", "TransitionBlueB/Angle",
                        0.0);

        private static final TunableNumber transitionBlueC_X = new TunableNumber("Field", "TransitionBlueC/X", 3.270);
        private static final TunableNumber transitionBlueC_Y = new TunableNumber("Field", "TransitionBlueC/Y", 7.410);
        private static final TunableNumber transitionBlueC_Angle = new TunableNumber("Field", "TransitionBlueC/Angle",
                        0.0);

        private static final TunableNumber transitionBlueD_X = new TunableNumber("Field", "TransitionBlueD/X", 5.700);
        private static final TunableNumber transitionBlueD_Y = new TunableNumber("Field", "TransitionBlueD/Y", 7.410);
        private static final TunableNumber transitionBlueD_Angle = new TunableNumber("Field", "TransitionBlueD/Angle",
                        0.0);

        // TRANSITION POINTS (Red Alliance has A, B, C, D)
        private static final TunableNumber transitionRedA_X = new TunableNumber("Field", "TransitionRedA/X", 13.400);
        private static final TunableNumber transitionRedA_Y = new TunableNumber("Field", "TransitionRedA/Y", 7.470);
        private static final TunableNumber transitionRedA_Angle = new TunableNumber("Field", "TransitionRedA/Angle",
                        180.0);

        private static final TunableNumber transitionRedB_X = new TunableNumber("Field", "TransitionRedB/X", 10.800);
        private static final TunableNumber transitionRedB_Y = new TunableNumber("Field", "TransitionRedB/Y", 7.470);
        private static final TunableNumber transitionRedB_Angle = new TunableNumber("Field", "TransitionRedB/Angle",
                        180.0);

        private static final TunableNumber transitionRedC_X = new TunableNumber("Field", "TransitionRedC/X", 13.400);
        private static final TunableNumber transitionRedC_Y = new TunableNumber("Field", "TransitionRedC/Y", 0.650);
        private static final TunableNumber transitionRedC_Angle = new TunableNumber("Field", "TransitionRedC/Angle",
                        180.0);

        private static final TunableNumber transitionRedD_X = new TunableNumber("Field", "TransitionRedD/X", 10.800);
        private static final TunableNumber transitionRedD_Y = new TunableNumber("Field", "TransitionRedD/Y", 0.650);
        private static final TunableNumber transitionRedD_Angle = new TunableNumber("Field", "TransitionRedD/Angle",
                        180.0);

        // OUTPOST POSITIONS (Per Alliance)
        private static final TunableNumber outpostBlueX = new TunableNumber("Field", "OutpostBlue/X", 15.0);
        private static final TunableNumber outpostBlueY = new TunableNumber("Field", "OutpostBlue/Y", 7.0);
        private static final TunableNumber outpostBlueAngle = new TunableNumber("Field", "OutpostBlue/Angle", 180.0);

        private static final TunableNumber outpostRedX = new TunableNumber("Field", "OutpostRed/X", 1.49);
        private static final TunableNumber outpostRedY = new TunableNumber("Field", "OutpostRed/Y", 7.0);
        private static final TunableNumber outpostRedAngle = new TunableNumber("Field", "OutpostRed/Angle", 0.0);

        // =====================================================================
        // FEED PASS POSITIONS (Auto Collection Paths)
        // =====================================================================
        
        // --- BLUE ALLIANCE FEED PASS (Diagonal Paths for Maximum Collection) ---
        // Left Path (Y High to Low Diagonal)
        private static final TunableNumber feedStartBlueLeftX = new TunableNumber("Field", "FeedStartBlueLeft/X", 10.0);
        private static final TunableNumber feedStartBlueLeftY = new TunableNumber("Field", "FeedStartBlueLeft/Y", 7.0);
        private static final TunableNumber feedStartBlueLeftAngle = new TunableNumber("Field", "FeedStartBlueLeft/Angle", 0.0);
        
        private static final TunableNumber feedStopBlueLeftX = new TunableNumber("Field", "FeedStopBlueLeft/X", 6.0);
        private static final TunableNumber feedStopBlueLeftY = new TunableNumber("Field", "FeedStopBlueLeft/Y", 1.0);
        private static final TunableNumber feedStopBlueLeftAngle = new TunableNumber("Field", "FeedStopBlueLeft/Angle", 0.0);

        // Right Path (Y Low to High Diagonal)
        private static final TunableNumber feedStartBlueRightX = new TunableNumber("Field", "FeedStartBlueRight/X", 10.0);
        private static final TunableNumber feedStartBlueRightY = new TunableNumber("Field", "FeedStartBlueRight/Y", 1.0);
        private static final TunableNumber feedStartBlueRightAngle = new TunableNumber("Field", "FeedStartBlueRight/Angle", 0.0);
        
        private static final TunableNumber feedStopBlueRightX = new TunableNumber("Field", "FeedStopBlueRight/X", 6.0);
        private static final TunableNumber feedStopBlueRightY = new TunableNumber("Field", "FeedStopBlueRight/Y", 7.0);
        private static final TunableNumber feedStopBlueRightAngle = new TunableNumber("Field", "FeedStopBlueRight/Angle", 0.0);

        // --- RED ALLIANCE FEED PASS (Diagonal Paths for Maximum Collection) ---
        // Left Path (Y High to Low Diagonal)
        private static final TunableNumber feedStartRedLeftX = new TunableNumber("Field", "FeedStartRedLeft/X", 6.0);
        private static final TunableNumber feedStartRedLeftY = new TunableNumber("Field", "FeedStartRedLeft/Y", 7.0);
        private static final TunableNumber feedStartRedLeftAngle = new TunableNumber("Field", "FeedStartRedLeft/Angle", 180.0);
        
        private static final TunableNumber feedStopRedLeftX = new TunableNumber("Field", "FeedStopRedLeft/X", 10.0);
        private static final TunableNumber feedStopRedLeftY = new TunableNumber("Field", "FeedStopRedLeft/Y", 1.0);
        private static final TunableNumber feedStopRedLeftAngle = new TunableNumber("Field", "FeedStopRedLeft/Angle", 180.0);

        // Right Path (Y Low to High Diagonal)
        private static final TunableNumber feedStartRedRightX = new TunableNumber("Field", "FeedStartRedRight/X", 6.0);
        private static final TunableNumber feedStartRedRightY = new TunableNumber("Field", "FeedStartRedRight/Y", 1.0);
        private static final TunableNumber feedStartRedRightAngle = new TunableNumber("Field", "FeedStartRedRight/Angle", 180.0);
        
        private static final TunableNumber feedStopRedRightX = new TunableNumber("Field", "FeedStopRedRight/X", 10.0);
        private static final TunableNumber feedStopRedRightY = new TunableNumber("Field", "FeedStopRedRight/Y", 7.0);
        private static final TunableNumber feedStopRedRightAngle = new TunableNumber("Field", "FeedStopRedRight/Angle", 180.0);

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
         * Gets Transition Point A for Blue Alliance
         */
        public static Pose2d getTransitionPointBlueA() {
                return new Pose2d(transitionBlueA_X.get(), transitionBlueA_Y.get(),
                                Rotation2d.fromDegrees(transitionBlueA_Angle.get()));
        }

        public static Pose2d getTransitionPointBlueB() {
                return new Pose2d(transitionBlueB_X.get(), transitionBlueB_Y.get(),
                                Rotation2d.fromDegrees(transitionBlueB_Angle.get()));
        }

        public static Pose2d getTransitionPointBlueC() {
                return new Pose2d(transitionBlueC_X.get(), transitionBlueC_Y.get(),
                                Rotation2d.fromDegrees(transitionBlueC_Angle.get()));
        }

        public static Pose2d getTransitionPointBlueD() {
                return new Pose2d(transitionBlueD_X.get(), transitionBlueD_Y.get(),
                                Rotation2d.fromDegrees(transitionBlueD_Angle.get()));
        }

        public static Pose2d getTransitionPointRedA() {
                return new Pose2d(transitionRedA_X.get(), transitionRedA_Y.get(),
                                Rotation2d.fromDegrees(transitionRedA_Angle.get()));
        }

        public static Pose2d getTransitionPointRedB() {
                return new Pose2d(transitionRedB_X.get(), transitionRedB_Y.get(),
                                Rotation2d.fromDegrees(transitionRedB_Angle.get()));
        }

        public static Pose2d getTransitionPointRedC() {
                return new Pose2d(transitionRedC_X.get(), transitionRedC_Y.get(),
                                Rotation2d.fromDegrees(transitionRedC_Angle.get()));
        }

        public static Pose2d getTransitionPointRedD() {
                return new Pose2d(transitionRedD_X.get(), transitionRedD_Y.get(),
                                Rotation2d.fromDegrees(transitionRedD_Angle.get()));
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

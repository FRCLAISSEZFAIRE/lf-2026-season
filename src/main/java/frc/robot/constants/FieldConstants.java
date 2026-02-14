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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * Field Constants for FRC 2026 Rebuilt Game.
 * Loads field layout from "2026-rebuilt-welded.json" and calculates centroids.
 */
public final class FieldConstants {

        public static final double kFieldLengthMeters = 16.49;
        public static final double kFieldWidthMeters = 8.10;

        // Layout
        public static AprilTagFieldLayout fieldLayout;

        // Tag Lists
        private static final List<Integer> RED_HUB_TAGS = List.of(2, 3, 4, 5, 8, 9, 10, 11);
        private static final List<Integer> BLUE_HUB_TAGS = List.of(18, 19, 20, 21, 24, 25, 26, 27);
        private static final List<Integer> RED_HANG_TAGS = List.of(15, 16);
        private static final List<Integer> BLUE_HANG_TAGS = List.of(31, 32);

        // Calculated Centers (Cached)
        public static final Translation2d RED_HUB_CENTER;
        public static final Translation2d BLUE_HUB_CENTER;
        public static final Translation2d RED_HANG_CENTER;
        public static final Translation2d BLUE_HANG_CENTER;

        // === LEGACY CONSTANTS (Restored & Linked) ===
        public static final double kHubHeightMeters = 1.83;
        public static final Pose2d kBlueHubPose;
        public static final Pose2d kRedHubPose;

        // Speaker Positions (Linked to Calculated Centers)

        public static final Pose2d kBlueStartPose = new Pose2d(2.0, 4.05, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedStartPose = new Pose2d(14.49, 4.05, Rotation2d.fromDegrees(180));
        public static final Pose2d kTestStartPose = new Pose2d(8.245, 4.05, Rotation2d.fromDegrees(0));

        public static final double kBlueAllianceZoneMaxX = 4.0;
        public static final double kRedAllianceZoneMinX = 12.49;

        public static final Translation2d kBlueFeedingTarget = new Translation2d(3.5, 4.05);
        public static final Translation2d kRedFeedingTarget = new Translation2d(12.99, 4.05);

        public static final double kIdealShootingDistanceMeters = 3.0;

        public static final Pose2d kBlueDepotPose = new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedDepotPose = new Pose2d(14.99, 1.0, Rotation2d.fromDegrees(180));

        public static final Pose2d kBlueOutpostPose = new Pose2d(15.0, 7.0, Rotation2d.fromDegrees(180));
        public static final Pose2d kRedOutpostPose = new Pose2d(1.49, 7.0, Rotation2d.fromDegrees(0));

        public static final Translation2d kTowerCenter = new Translation2d(8.245, 4.05);
        public static final double kTowerClimbHeightMeters = 1.5;

        public static final Pose2d[] kTowerClimbPoses = new Pose2d[] {
                        new Pose2d(kTowerCenter.getX() - 1.0, kTowerCenter.getY(), Rotation2d.fromDegrees(0)),
                        new Pose2d(kTowerCenter.getX(), kTowerCenter.getY(), Rotation2d.fromDegrees(0)),
                        new Pose2d(kTowerCenter.getX() + 1.0, kTowerCenter.getY(), Rotation2d.fromDegrees(0))
        };

        public static final Pose2d[] kSourcePoses = new Pose2d[] {
                        kBlueDepotPose,
                        kBlueOutpostPose
        };

        public static final double kFuelDiameterMeters = 0.15;

        public static final List<Translation2d> kKeepOutZones = new ArrayList<>();

        static {
                Translation2d redHub = new Translation2d();
                Translation2d blueHub = new Translation2d();
                Translation2d redHang = new Translation2d();
                Translation2d blueHang = new Translation2d();

                try {
                        // Load Layout
                        String filename = "2026-rebuilt-welded.json";
                        Path path = Filesystem.getDeployDirectory().toPath().resolve(filename);
                        fieldLayout = new AprilTagFieldLayout(path);

                        // Calculate Centers
                        redHub = calculateAverageCentroid(RED_HUB_TAGS);
                        blueHub = calculateAverageCentroid(BLUE_HUB_TAGS);
                        redHang = calculateAverageCentroid(RED_HANG_TAGS);
                        blueHang = calculateAverageCentroid(BLUE_HANG_TAGS);

                        System.out.println(
                                        "[FieldConstants] Layout Loaded. RedHub: " + redHub + ", BlueHub: " + blueHub);

                } catch (IOException e) {
                        System.err.println("[FieldConstants] ERROR: Could not load " + "2026-rebuilt-welded.json");
                        e.printStackTrace();
                        // Fallback to hardcoded constants if file missing (avoid crash)
                        redHub = new Translation2d(15.49, 4.05);
                        blueHub = new Translation2d(1.0, 4.05);
                        redHang = new Translation2d(11.0, 4.05); // Approximate
                        blueHang = new Translation2d(5.0, 4.05); // Approximate
                }

                RED_HUB_CENTER = redHub;
                BLUE_HUB_CENTER = blueHub;
                RED_HANG_CENTER = redHang;
                BLUE_HANG_CENTER = blueHang;

                kBlueHubPose = new Pose2d(BLUE_HUB_CENTER, Rotation2d.fromDegrees(0));
                kRedHubPose = new Pose2d(RED_HUB_CENTER, Rotation2d.fromDegrees(180));

                // kKeepOutZones.add(kTowerCenter);
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
         * Gets the Pass Target (Feeding Station area) for the specified alliance.
         */
        public static Translation2d getPassTarget(Optional<Alliance> alliance) {
                if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                        return kRedFeedingTarget;
                }
                return kBlueFeedingTarget;
        }

        private FieldConstants() {
        }
}

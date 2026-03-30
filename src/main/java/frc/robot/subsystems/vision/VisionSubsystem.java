package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Enhanced Vision Subsystem with dual Limelight pose estimation.
 * 
 * <h2>Camera Configuration:</h2>
 * <ul>
 * <li><b>limelight-left:</b> Limelight 4, rear-left corner, 30° outward -
 * MegaTag 2</li>
 * <li><b>limelight-right:</b> Limelight 3, rear-right corner, 30° outward -
 * MegaTag 2</li>
 * 
 * </ul>
 * 
 * <h2>Features:</h2>
 * <ul>
 * <li>Dual Limelight MegaTag2 pose fusion</li>
 * <li>Tuning Table toggle (default: OFF)</li>
 * <li>Dynamic confidence matrix based on tag count, distance, rotation
 * rate</li>
 * <li>AdvantageKit telemetry support</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

    private final DriveSubsystem drive;

    // ==================== DASHBOARD TOGGLE ====================
    private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");
    private boolean visionEnabled = true; // DEFAULT: ON - enabled by default for pose estimation

    // ==================== TELEMETRY CACHE ====================
    // Left camera
    private boolean leftHasValidPose = false;
    private int leftTagCount = 0;
    private double leftAvgTagDistance = 0.0;
    private Pose2d leftLatestPose = new Pose2d();

    // Right camera
    private boolean rightHasValidPose = false;
    private int rightTagCount = 0;
    private double rightAvgTagDistance = 0.0;
    private Pose2d rightLatestPose = new Pose2d();

    // Combined/fused
    private boolean hasValidPose = false;
    private int totalTagCount = 0;
    private Pose2d latestVisionPose = new Pose2d();
    private double latestTimestamp = 0.0;

    // ==================== CONSTRUCTOR ====================
    public VisionSubsystem(DriveSubsystem drive) {
        this.drive = drive;

        // Initialize Tuning Table entry
        tuningTable.getEntry("Vision/Enabled").setDefaultBoolean(true);

        // ==================== LIMELIGHT KAMERA POZİSYON AYARLARI ====================
        // Kamera offset'lerini Limelight'a gönder (robot merkezine göre).
        // Bu yapılmazsa Limelight kameranın (0,0,0)'da olduğunu varsayar ve
        // tüm pose tahminleri sistematik olarak kayar.
        // NOT: Limelight setCameraPose_RobotSpace parametreleri:
        //   forward (m), side (m), up (m), roll (°), pitch (°), yaw (°)
        LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.kLimelightLeft,
                VisionConstants.kLeftCameraToRobot.getTranslation().getX(),   // forward
                VisionConstants.kLeftCameraToRobot.getTranslation().getY(),   // side
                VisionConstants.kLeftCameraToRobot.getTranslation().getZ(),   // up
                Math.toDegrees(VisionConstants.kLeftCameraToRobot.getRotation().getX()), // roll
                Math.toDegrees(VisionConstants.kLeftCameraToRobot.getRotation().getY()), // pitch
                Math.toDegrees(VisionConstants.kLeftCameraToRobot.getRotation().getZ())  // yaw
        );
        LimelightHelpers.setCameraPose_RobotSpace(
                VisionConstants.kLimelightRight,
                VisionConstants.kRightCameraToRobot.getTranslation().getX(),
                VisionConstants.kRightCameraToRobot.getTranslation().getY(),
                VisionConstants.kRightCameraToRobot.getTranslation().getZ(),
                Math.toDegrees(VisionConstants.kRightCameraToRobot.getRotation().getX()),
                Math.toDegrees(VisionConstants.kRightCameraToRobot.getRotation().getY()),
                Math.toDegrees(VisionConstants.kRightCameraToRobot.getRotation().getZ())
        );

        // Pipeline index'lerini garanti altına al (reboot sonrası yanlış pipeline'da kalabilir)
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightLeft, VisionConstants.kAprilTagPipelineIndex);
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightRight, VisionConstants.kAprilTagPipelineIndex);

        System.out.println("[VisionSubsystem] Kamera pozisyonları ve pipeline'lar Limelight'a gönderildi.");
    }

    // ==================== PERIODIC ====================
    @Override
    public void periodic() {
        // Check tuning table toggle FIRST
        visionEnabled = tuningTable.getEntry("Vision/Enabled").getBoolean(true);
        Logger.recordOutput("Tuning/Vision/Enabled", visionEnabled);

        // Early return if vision disabled
        if (!visionEnabled) {
            hasValidPose = false;
            leftHasValidPose = false;
            rightHasValidPose = false;
            totalTagCount = 0;
            return;
        }

        // Get gyro data for MegaTag 2 sync (shared for both cameras)
        double gyroYawDegrees = drive.getRotation2d().getDegrees();
        double gyroRateDegPerSec = Units.radiansToDegrees(drive.getGyroVelocityRadPerSec());

        // Process both Limelights
        updatePoseFromCamera(VisionConstants.kLimelightLeft, gyroYawDegrees, gyroRateDegPerSec, true);
        updatePoseFromCamera(VisionConstants.kLimelightRight, gyroYawDegrees, gyroRateDegPerSec, false);

        // Update combined stats
        totalTagCount = leftTagCount + rightTagCount;
        hasValidPose = leftHasValidPose || rightHasValidPose;

        // AdvantageKit Logging
        // AdvantageKit Logging
        Logger.recordOutput("Tuning/Vision/HasValidPose", hasValidPose);
        Logger.recordOutput("Tuning/Vision/TotalTagCount", totalTagCount);
        Logger.recordOutput("Tuning/Vision/LeftCamera/Valid", leftHasValidPose);
        Logger.recordOutput("Tuning/Vision/LeftCamera/TagCount", leftTagCount);
        Logger.recordOutput("Tuning/Vision/LeftCamera/AvgDist", leftAvgTagDistance);
        Logger.recordOutput("Tuning/Vision/LeftCamera/Pose", leftLatestPose);
        Logger.recordOutput("Tuning/Vision/RightCamera/Valid", rightHasValidPose);
        Logger.recordOutput("Tuning/Vision/RightCamera/TagCount", rightTagCount);
        Logger.recordOutput("Tuning/Vision/RightCamera/AvgDist", rightAvgTagDistance);
        Logger.recordOutput("Tuning/Vision/RightCamera/Pose", rightLatestPose);
    }

    // ==================== POSE ESTIMATION ====================

    /**
     * Process pose estimation from a single Limelight camera.
     * Uses MegaTag 2 with NavX synchronization.
     * 
     * @param cameraName        Limelight NetworkTables name
     * @param gyroYawDeg        Current gyro yaw in degrees
     * @param gyroRateDegPerSec Current gyro angular velocity
     * @param isLeftCamera      true for left camera, false for right
     */
    private void updatePoseFromCamera(String cameraName, double gyroYawDeg,
            double gyroRateDegPerSec, boolean isLeftCamera) {
        // Reset validity
        if (isLeftCamera) {
            leftHasValidPose = false;
            leftTagCount = 0;
        } else {
            rightHasValidPose = false;
            rightTagCount = 0;
        }

        // NavX Sync (MegaTag 2 requirement)
        LimelightHelpers.SetRobotOrientation(
                cameraName,
                gyroYawDeg,
                gyroRateDegPerSec,
                0, 0, 0, 0);

        // Get MegaTag 2 pose estimate (SADECE MegaTag2 — MegaTag1 fallback yok)
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
        if (mt2 == null || mt2.pose == null) {
            Logger.recordOutput("Tuning/Vision/" + (isLeftCamera ? "Left" : "Right") + "/RejectReason", "NullMT2");
            return;
        }

        // Update cache
        if (isLeftCamera) {
            leftTagCount = mt2.tagCount;
            leftAvgTagDistance = mt2.avgTagDist;
            leftLatestPose = mt2.pose;
        } else {
            rightTagCount = mt2.tagCount;
            rightAvgTagDistance = mt2.avgTagDist;
            rightLatestPose = mt2.pose;
        }

        // Validation: High gyro rate rejection
        if (Math.abs(gyroRateDegPerSec) > VisionConstants.kMaxGyroRateForVision) {
            Logger.recordOutput("Tuning/Vision/" + (isLeftCamera ? "Left" : "Right") + "/RejectReason", "HighGyroRate");
            return;
        }

        // Validation: No tags
        if (mt2.tagCount == 0) {
            Logger.recordOutput("Tuning/Vision/" + (isLeftCamera ? "Left" : "Right") + "/RejectReason", "NoTags");
            return;
        }

        // Validation: Out of field bounds — Saha dışı saçma pozisyonları reddet
        if (mt2.pose.getX() < -0.5 || mt2.pose.getX() > 17.0 ||
                mt2.pose.getY() < -0.5 || mt2.pose.getY() > 9.0) {
            Logger.recordOutput("Tuning/Vision/" + (isLeftCamera ? "Left" : "Right") + "/RejectReason", "OutOfField");
            return;
        }

        // Calculate dynamic standard deviations
        double xyStdev = calculateXYStdDev(mt2, gyroRateDegPerSec);
        double thetaStdev = calculateThetaStdDev(mt2, gyroRateDegPerSec);

        // Add vision measurement to drive pose estimator
        drive.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds,
                VecBuilder.fill(xyStdev, xyStdev, thetaStdev));

        // Mark as valid
        if (isLeftCamera) {
            leftHasValidPose = true;
        } else {
            rightHasValidPose = true;
        }

        latestVisionPose = mt2.pose;
        latestTimestamp = mt2.timestampSeconds;

        Logger.recordOutput("Tuning/Vision/" + (isLeftCamera ? "Left" : "Right") + "/RejectReason", "None");
        Logger.recordOutput("Tuning/Vision/" + (isLeftCamera ? "Left" : "Right") + "/XYStdDev", xyStdev);
        Logger.recordOutput("Tuning/Vision/" + (isLeftCamera ? "Left" : "Right") + "/ThetaStdDev", thetaStdev);
    }

    /**
     * Calculate XY position standard deviation based on conditions.
     */
    private double calculateXYStdDev(PoseEstimate mt2, double gyroRateDegPerSec) {
        double stdev = 0.5; // Base value

        // High rotation increases uncertainty
        if (Math.abs(gyroRateDegPerSec) > 100.0) {
            stdev += 0.5;
        }

        // Multiple tags increase confidence
        if (mt2.tagCount > 1) {
            stdev -= 0.3;
        }

        // Distance affects uncertainty
        if (mt2.avgTagDist > VisionConstants.kMaxReliableTagDistance) {
            stdev += 0.5;
        } else if (mt2.avgTagDist > VisionConstants.kHighUncertaintyTagDistance) {
            stdev += 0.2;
        }

        return Math.max(0.1, stdev);
    }

    /**
     * Calculate theta (rotation) standard deviation based on conditions.
     */
    private double calculateThetaStdDev(PoseEstimate mt2, double gyroRateDegPerSec) {
        // MEGATAG 2 REQUIREMENT:
        // We MUST trust the Gyro for correlation.
        // The vision estimate's rotation is essentially ignored for state update,
        // but used by the MT2 algorithm internally for X/Y calculation.
        // We set this to Infinity (Double.MAX_VALUE) so the Pose Estimator
        // never uses the vision rotation to override the Gyro.
        return 9999999.0;
    }

    /**
     * Check if vision is currently enabled via dashboard.
     */
    public boolean isVisionEnabled() {
        return visionEnabled;
    }

    /**
     * Check if we have a valid pose estimate from either camera.
     */
    public boolean hasValidPoseEstimate() {
        return hasValidPose;
    }

    /**
     * Get total number of AprilTags currently visible across all cameras.
     */
    public int getTotalTagCount() {
        return totalTagCount;
    }

    /**
     * Get the latest fused vision pose estimate.
     */
    public Pose2d getLatestVisionPose() {
        return latestVisionPose;
    }

    /**
     * Get the timestamp of the latest valid vision measurement.
     */
    public double getLatestTimestamp() {
        return latestTimestamp;
    }

    // Legacy compatibility
    public int getTagCount() {
        return totalTagCount;
    }

    public double getAverageTagDistance() {
        return (leftAvgTagDistance + rightAvgTagDistance) / 2.0;
    }
}
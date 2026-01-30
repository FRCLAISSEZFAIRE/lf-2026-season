package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Gelişmiş Vision Subsystem.
 * 
 * <h2>Kamera Yapılandırması:</h2>
 * <ul>
 * <li>Limelight 3 (MegaTag 2): Pose Estimation & NavX Sync</li>
 * <li>Limelight 3A: Object Detection (Intake)</li>
 * </ul>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>MegaTag2 ile hassas pose estimation</li>
 * <li>Dinamik güven matrisi (tag sayısı, mesafe, dönüş hızına göre)</li>
 * <li>AdvantageKit telemetry desteği</li>
 * <li>Multi-tag lokalizasyon</li>
 * </ul>
 */
public class VisionSubsystem extends SubsystemBase {

    private final DriveSubsystem drive;

    // Kamera İsimleri
    private final String poseCameraName;
    private final String intakeCameraName;

    // Telemetry cache
    private boolean hasValidPose = false;
    private int tagCount = 0;
    private double avgTagDistance = 0.0;
    private Pose2d latestVisionPose = new Pose2d();
    private double latestTimestamp = 0.0;

    public VisionSubsystem(DriveSubsystem drive) {
        this.drive = drive;
        this.poseCameraName = VisionConstants.kLimelightName;
        this.intakeCameraName = "limelight-object";
    }

    @Override
    public void periodic() {
        // Pose Estimation (MegaTag 2)
        updatePoseEstimation();

        // AdvantageKit Logging
        Logger.recordOutput("Vision/HasValidPose", hasValidPose);
        Logger.recordOutput("Vision/TagCount", tagCount);
        Logger.recordOutput("Vision/AvgTagDistance", avgTagDistance);
        Logger.recordOutput("Vision/LatestPose", latestVisionPose);
        Logger.recordOutput("Vision/Timestamp", latestTimestamp);
    }

    // ===========================================================================
    // POSE ESTIMATION
    // ===========================================================================

    /**
     * MegaTag2 ile pose estimation güncelleme.
     * NavX sync ve dinamik güven matrisi kullanır.
     */
    private void updatePoseEstimation() {
        hasValidPose = false;

        // NavX Sync (MegaTag 2 için zorunlu)
        double gyroYawDegrees = drive.getRotation2d().getDegrees();
        double gyroRateDegPerSec = Units.radiansToDegrees(drive.getGyroVelocityRadPerSec());

        LimelightHelpers.SetRobotOrientation(
                poseCameraName,
                gyroYawDegrees,
                gyroRateDegPerSec,
                0, 0, 0, 0);

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(poseCameraName);

        // Null kontrolü
        if (mt2 == null) {
            tagCount = 0;
            return;
        }

        tagCount = mt2.tagCount;
        avgTagDistance = mt2.avgTagDist;
        latestVisionPose = mt2.pose;
        latestTimestamp = mt2.timestampSeconds;

        // Geçersiz veri kontrolü - Yüksek dönüş hızında güvenme
        if (Math.abs(gyroRateDegPerSec) > 720.0) {
            Logger.recordOutput("Vision/RejectReason", "HighGyroRate");
            return;
        }

        // Tag yoksa güvenme
        if (mt2.tagCount == 0) {
            Logger.recordOutput("Vision/RejectReason", "NoTags");
            return;
        }

        // Saha dışı kontrolü
        if (mt2.pose.getX() < -0.5 || mt2.pose.getX() > 17.0 ||
                mt2.pose.getY() < -0.5 || mt2.pose.getY() > 9.0) {
            Logger.recordOutput("Vision/RejectReason", "OutOfField");
            return;
        }

        // Dinamik Standard Deviation hesaplama
        double xyStdev = calculateXYStdDev(mt2, gyroRateDegPerSec);
        double thetaStdev = calculateThetaStdDev(mt2, gyroRateDegPerSec);

        // Vision measurement ekle
        drive.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds,
                VecBuilder.fill(xyStdev, xyStdev, thetaStdev));

        hasValidPose = true;
        Logger.recordOutput("Vision/RejectReason", "None");
        Logger.recordOutput("Vision/XYStdDev", xyStdev);
        Logger.recordOutput("Vision/ThetaStdDev", thetaStdev);
    }

    /**
     * XY pozisyon güvenilirliğini hesapla.
     */
    private double calculateXYStdDev(PoseEstimate mt2, double gyroRateDegPerSec) {
        double stdev = 0.5; // Base value

        // Dönüş hızı artarsa güvenilirlik azalır
        if (Math.abs(gyroRateDegPerSec) > 100.0) {
            stdev += 0.5;
        }

        // Birden fazla tag varsa güvenilirlik artar
        if (mt2.tagCount > 1) {
            stdev -= 0.3;
        }

        // Tag uzaktaysa güvenilirlik azalır
        if (mt2.avgTagDist > 4.0) {
            stdev += 0.5;
        } else if (mt2.avgTagDist > 3.0) {
            stdev += 0.2;
        }

        return Math.max(0.1, stdev);
    }

    /**
     * Açısal (theta) güvenilirliğini hesapla.
     */
    private double calculateThetaStdDev(PoseEstimate mt2, double gyroRateDegPerSec) {
        double stdev = 0.5; // Base value

        // Dönüş hızı artarsa güvenilirlik çok azalır
        if (Math.abs(gyroRateDegPerSec) > 100.0) {
            stdev += 1.0;
        }

        // Birden fazla tag varsa güvenilirlik artar
        if (mt2.tagCount > 1) {
            stdev -= 0.3;
        }

        // Tag uzaktaysa güvenilirlik azalır
        if (mt2.avgTagDist > 4.0) {
            stdev += 0.5;
        }

        return Math.max(0.1, stdev);
    }

    // ===========================================================================
    // GAME PIECE DETECTION (Limelight 3A)
    // ===========================================================================

    /**
     * Intake kamerasında Fuel var mı?
     */
    public boolean hasFuel() {
        return LimelightHelpers.getTV(intakeCameraName);
    }

    /**
     * Fuel'in X açısı (yaw).
     */
    public double getFuelYaw() {
        return LimelightHelpers.getTX(intakeCameraName);
    }

    /**
     * Fuel'in Y açısı (pitch).
     */
    public double getFuelPitch() {
        return LimelightHelpers.getTY(intakeCameraName);
    }

    /**
     * Fuel'in görüntüdeki alanı (yakınlık göstergesi).
     */
    public double getFuelArea() {
        return LimelightHelpers.getTA(intakeCameraName);
    }

    /**
     * Pipeline değiştir.
     */
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(intakeCameraName, pipelineIndex);
    }

    // ===========================================================================
    // GETTERS
    // ===========================================================================

    public boolean hasValidPoseEstimate() {
        return hasValidPose;
    }

    public int getTagCount() {
        return tagCount;
    }

    public double getAverageTagDistance() {
        return avgTagDistance;
    }

    public Pose2d getLatestVisionPose() {
        return latestVisionPose;
    }
}
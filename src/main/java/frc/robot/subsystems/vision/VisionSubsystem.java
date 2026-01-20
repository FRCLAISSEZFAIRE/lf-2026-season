package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drive.DriveSubsystem;

/**
 * Gelişmiş Vision Subsystem (Dual Camera Support).
 * - Limelight 3 (MegaTag 2): Pose Estimation & NavX Sync
 * - Limelight 3A: Object Detection (Intake)
 */
public class VisionSubsystem extends SubsystemBase {
    private final DriveSubsystem drive;

    // Kamera İsimleri (NetworkTables)
    // Limelight 3 (Pose Estimation & MegaTag 2)
    private final String kPoseCameraName = "limelight";
    // Limelight 3A (Object Detection - Intake) - Kullanıcı "limelight 3a" olarak
    // belirtti
    private final String kIntakeCameraName = "limelight-3a";

    public VisionSubsystem(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void periodic() {
        // =========================================================================
        // 1. Pose Estimation Logic (Limelight 3 - MegaTag 2)
        // =========================================================================
        updatePoseEstimation();
    }

    private void updatePoseEstimation() {
        // NavX Sync (MegaTag 2 için zorunlu)
        double gyroYawDegrees = drive.getRotation2d().getDegrees();
        double gyroRateDegPerSec = Units.radiansToDegrees(drive.getGyroVelocityRadPerSec());

        LimelightHelpers.SetRobotOrientation(
                kPoseCameraName,
                gyroYawDegrees,
                gyroRateDegPerSec,
                0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kPoseCameraName);

        // Null kontrolü (Kamera bağlı değilse veya veri yoksa)
        if (mt2 == null)
            return;

        // Geçersiz veya güvensiz veri kontrolü
        if (Math.abs(gyroRateDegPerSec) > 720.0)
            return;
        if (mt2.tagCount == 0)
            return;

        // Saha Dışı Kontrolü
        if (mt2.pose.getX() < 0 || mt2.pose.getX() > 17 || mt2.pose.getY() < 0 || mt2.pose.getY() > 9)
            return;

        double timestamp = mt2.timestampSeconds;

        // Dynamic Standard Deviations (Trust Matrix)
        double xyStdev = 0.5;
        double thetaStdev = 0.5;

        if (Math.abs(gyroRateDegPerSec) > 100.0) {
            xyStdev += 0.5;
            thetaStdev += 1.0;
        }

        if (mt2.tagCount > 1) {
            xyStdev -= 0.3;
            thetaStdev -= 0.3;
        }

        if (mt2.avgTagDist > 4.0) {
            xyStdev += 0.5;
            thetaStdev += 0.5;
        }

        xyStdev = Math.max(0.1, xyStdev);
        thetaStdev = Math.max(0.1, thetaStdev);

        drive.addVisionMeasurement(
                mt2.pose,
                timestamp,
                VecBuilder.fill(xyStdev, xyStdev, thetaStdev));
    }

    // =========================================================================
    // Intake & Game Piece Detection (Limelight 3A - Object Detection)
    // =========================================================================

    public void setPipeline(int pipelineIndex) {
        // Intake kamerasının pipeline'ını değiştir
        LimelightHelpers.setPipelineIndex(kIntakeCameraName, pipelineIndex);
    }

    public boolean hasGamePiece() {
        // Intake kamerasında nesne var mı?
        return LimelightHelpers.getTV(kIntakeCameraName);
    }

    public double getGamePieceYaw() {
        // Intake kamerasındaki nesnenin açısı
        return LimelightHelpers.getTX(kIntakeCameraName);
    }
}
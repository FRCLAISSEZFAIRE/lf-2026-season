package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers; // Limelight kütüphanesi
import frc.robot.constants.VisionConstants;

public class VisionIOLimelight implements VisionIO {

    public VisionIOLimelight() {
        // Limelight başlangıç ayarları (Pipeline 0 genelde AprilTag'dir)
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightName, 0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);

        // Eğer geçerli bir tag görüyorsa
        if (limelightMeasurement != null && limelightMeasurement.tagCount > 0) {
            inputs.hasTarget = true;
            inputs.estimatedPose = limelightMeasurement.pose;
            inputs.timestamp = limelightMeasurement.timestampSeconds;
            inputs.tagCount = limelightMeasurement.tagCount;
            inputs.avgTagDist = limelightMeasurement.avgTagDist;
        } else {
            inputs.hasTarget = false;
            // Eski veriyi koru veya boşalt
        }
    }
}
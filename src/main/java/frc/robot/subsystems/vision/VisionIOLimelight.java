package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.LimelightHelpers;
import frc.robot.constants.VisionConstants;

public class VisionIOLimelight implements VisionIO {

    private int currentPipeline = VisionConstants.kAprilTagPipelineIndex;

    public VisionIOLimelight() {
        // Limelight başlangıç ayarları
        setPipeline(currentPipeline);
    }

    @Override
    public void setPipeline(int pipelineIndex) {
        currentPipeline = pipelineIndex;
        LimelightHelpers.setPipelineIndex(VisionConstants.kLimelightName, pipelineIndex);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // 1. AprilTag / Pose Estimate (MegaTag2)
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers
                .getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.kLimelightName);

        if (limelightMeasurement != null && limelightMeasurement.tagCount > 0) {
            inputs.hasTarget = true;
            inputs.estimatedPose = limelightMeasurement.pose;
            inputs.timestamp = limelightMeasurement.timestampSeconds;
            inputs.tagCount = limelightMeasurement.tagCount;
            inputs.avgTagDist = limelightMeasurement.avgTagDist;
        } else {
            inputs.hasTarget = false;
            inputs.tagCount = 0;
        }

        // 2. Game Piece Detection (Pipeline dependent)
        if (currentPipeline == VisionConstants.kGamePiecePipelineIndex) {
            boolean tv = LimelightHelpers.getTV(VisionConstants.kLimelightName);
            double tx = LimelightHelpers.getTX(VisionConstants.kLimelightName);
            
            inputs.hasGamePiece = tv;
            inputs.gamePieceYaw = tx;
        } else {
            inputs.hasGamePiece = false;
            inputs.gamePieceYaw = 0.0;
        }
    }
}
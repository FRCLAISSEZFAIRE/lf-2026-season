package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelight implements VisionIO {
    private final String name;

    public VisionIOLimelight(String name) {
        this.name = name;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.isConnected = NetworkTableInstance.getDefault().getTable(name).getEntry("tv").exists();

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        if (mt2 != null && mt2.pose != null && mt2.tagCount > 0) {
            inputs.hasData = true;
            inputs.pose = mt2.pose;
            inputs.timestampSeconds = mt2.timestampSeconds;
            inputs.tagCount = mt2.tagCount;
            inputs.avgTagDist = mt2.avgTagDist;
        } else {
            inputs.hasData = false;
        }
    }

    @Override
    public void setRobotOrientation(double yawDeg, double yawRateDegPerSec) {
        LimelightHelpers.SetRobotOrientation(name, yawDeg, yawRateDegPerSec, 0, 0, 0, 0);
    }
}

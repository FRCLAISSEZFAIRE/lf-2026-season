package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasData = false;
        public Pose2d pose = new Pose2d();
        public double timestampSeconds = 0.0;
        public int tagCount = 0;
        public double avgTagDist = 0.0;
        public boolean isConnected = false;
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default void setRobotOrientation(double yawDeg, double yawRateDegPerSec) {}
}

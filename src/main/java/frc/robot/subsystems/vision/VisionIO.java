package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public Pose2d estimatedPose = new Pose2d(); // Tahmini Konum
        public double timestamp = 0.0; // Görüntünün çekildiği an
        public int tagCount = 0; // Kaç tane AprilTag görüyor?
        public double avgTagDist = 0.0; // Taglere ortalama uzaklık
        
        // Game Piece Detection (Limelight 3A / ML)
        public boolean hasGamePiece = false;
        public double gamePieceYaw = 0.0; // tx (derece)
    }

    default void updateInputs(VisionIOInputs inputs) {}

    default void setPipeline(int pipelineIndex) {}
}
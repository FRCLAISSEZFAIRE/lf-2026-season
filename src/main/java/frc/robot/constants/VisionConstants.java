package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class VisionConstants {
    public static final String kLimelightName = "limelight";

    // Kameranın robot merkezine uzaklığı (RobotCenter -> Camera)
    public static final double kCameraXOffset = Units.inchesToMeters(10.0); 
    public static final double kCameraYOffset = 0.0;
    public static final double kCameraZOffset = Units.inchesToMeters(8.0);

    // Pipelines
    public static final int kAprilTagPipelineIndex = 0;
    public static final int kGamePiecePipelineIndex = 1;
}
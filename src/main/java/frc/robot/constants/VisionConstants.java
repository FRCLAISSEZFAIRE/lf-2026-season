package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Vision system constants for dual Limelight pose estimation and intake camera.
 * 
 * <h2>Camera Configuration:</h2>
 * <ul>
 * <li><b>limelight-left:</b> Rear-left corner, 45° outward - Pose Estimation
 * (MegaTag 2)</li>
 * <li><b>limelight-right:</b> Rear-right corner, 45° outward - Pose Estimation
 * (MegaTag 2)</li>
 * <li><b>intake-cam:</b> Near intake - Game Piece Detection (Color
 * Pipeline)</li>
 * </ul>
 */
public final class VisionConstants {

    // ==================== CAMERA NAMES ====================
    /** Left Limelight 3A - rear-left corner, pose estimation */
    public static final String kLimelightLeft = "limelight-left";

    /** Right Limelight 3 - rear-right corner, pose estimation */
    public static final String kLimelightRight = "limelight-right";

    /** Intake camera - game piece detection */
    public static final String kIntakeCamera = "intake-cam";

    // ==================== CAMERA TRANSFORMS ====================
    // Robot coordinate system: X = forward, Y = left, Z = up
    // Transforms are Robot Center → Camera

    // Physical measurements (adjust these to match your robot!)
    private static final double kCameraXBackOffset = Units.inchesToMeters(-12.0); // 12" behind center
    private static final double kCameraYSideOffset = Units.inchesToMeters(10.0); // 10" from center
    private static final double kCameraZHeight = Units.inchesToMeters(8.0); // 8" above ground
    private static final double kCameraPitchDegrees = -15.0; // Tilted down 15°

    /**
     * Left camera transform: rear-left corner, angled 45° outward (looking
     * back-left).
     * Yaw = -135° means camera faces back-left diagonal.
     */
    public static final Transform3d kLeftCameraToRobot = new Transform3d(
            new Translation3d(
                    kCameraXBackOffset, // X: behind robot center
                    kCameraYSideOffset, // Y: left of center
                    kCameraZHeight // Z: height
            ),
            new Rotation3d(
                    0, // Roll: 0
                    Math.toRadians(kCameraPitchDegrees), // Pitch: tilted down
                    Math.toRadians(-135) // Yaw: facing back-left (45° outward)
            ));

    /**
     * Right camera transform: rear-right corner, angled 45° outward (looking
     * back-right).
     * Yaw = 135° means camera faces back-right diagonal.
     */
    public static final Transform3d kRightCameraToRobot = new Transform3d(
            new Translation3d(
                    kCameraXBackOffset, // X: behind robot center
                    -kCameraYSideOffset, // Y: right of center (negative)
                    kCameraZHeight // Z: height
            ),
            new Rotation3d(
                    0, // Roll: 0
                    Math.toRadians(kCameraPitchDegrees), // Pitch: tilted down
                    Math.toRadians(135) // Yaw: facing back-right (45° outward)
            ));

    // ==================== PIPELINES ====================
    /** AprilTag detection pipeline index */
    public static final int kAprilTagPipelineIndex = 0;

    /** Game piece detection pipeline index */
    public static final int kGamePiecePipelineIndex = 1;

    // ==================== FILTERING THRESHOLDS ====================
    /** Maximum gyro rate (deg/sec) before rejecting vision data */
    public static final double kMaxGyroRateForVision = 720.0;

    /** Maximum tag distance (meters) before increasing uncertainty */
    public static final double kMaxReliableTagDistance = 4.0;

    /** Minimum tag distance threshold for high uncertainty */
    public static final double kHighUncertaintyTagDistance = 3.0;
}
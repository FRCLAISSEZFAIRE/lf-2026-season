package frc.robot.constants;

/**
 * Grabber alt sistemi için sabitler.
 * NEO 550 + Through Bore Encoder ile pozisyon kontrolü.
 */
public final class GrabberConstants {

    // --- MOTOR ID (RobotMap'ten) ---
    public static final int kGrabberMotorID = RobotMap.kGrabberMotorID;

    // --- ENCODER ---
    public static final double kEncoderPositionFactor = 360.0;

    // --- PID GAINS ---
    public static final double kP = 0.03;
    public static final double kI = 0.0;
    public static final double kD = 0.002;

    // --- OUTPUT LIMITS ---
    public static final double kMaxOutput = 0.4;
    public static final double kMinOutput = -0.4;

    // --- POSITIONS (Derece) ---
    public static final double kOpenPosition = 0.0;
    public static final double kClosedPosition = 45.0;

    // --- TOLERANCE ---
    public static final double kPositionTolerance = 2.0;

    // --- CURRENT LIMITS ---
    public static final int kCurrentLimit = 20;
}

package frc.robot.constants;

/**
 * Wrist alt sistemi için sabitler.
 * Arm ucunda ince açı ayarı.
 */
public final class WristConstants {

    // --- MOTOR ID (RobotMap'ten) ---
    public static final int kWristMotorID = RobotMap.kWristMotorID;

    // --- PID GAINS ---
    public static final double kP = 0.02;
    public static final double kI = 0.0;
    public static final double kD = 0.001;

    // --- OUTPUT LIMITS ---
    public static final double kMaxOutput = 0.3;
    public static final double kMinOutput = -0.3;

    // --- ANGLE LIMITS (Derece) ---
    public static final double kMinAngle = -45.0;
    public static final double kMaxAngle = 45.0;

    // --- PRESETS ---
    public static final double kCenterAngle = 0.0;
    public static final double kIntakeAngle = -30.0;
    public static final double kScoreAngle = 30.0;

    // --- TOLERANCE ---
    public static final double kAngleTolerance = 2.0;
}

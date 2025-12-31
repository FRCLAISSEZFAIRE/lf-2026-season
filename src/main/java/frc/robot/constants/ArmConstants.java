package frc.robot.constants;

/**
 * Arm alt sistemi için sabitler.
 * SparkMax + AbsoluteEncoder + PID
 */
public final class ArmConstants {

    // --- MOTOR ID (RobotMap'ten) ---
    public static final int kArmMotorID = RobotMap.kArmMotorID;

    // --- ENCODER ---
    public static final double kAngleEncoderPositionFactor = 360.0;

    // --- PID GAINS ---
    public static final double kAngleP = 0.02;
    public static final double kAngleI = 0.0;
    public static final double kAngleD = 0.001;

    // --- OUTPUT LIMITS ---
    public static final double kMaxOutput = 0.3;
    public static final double kMinOutput = -0.3;

    // --- ANGLE LIMITS (Derece) ---
    public static final double kAngleLowerLimitDegrees = 0.0;
    public static final double kAngleUpperLimitDegrees = 120.0;

    // --- PRESET POSITIONS (Derece) ---
    public static final double kStowedAngle = 10.0;
    public static final double kIntakeAngle = 0.0;
    public static final double kScoreL1Angle = 45.0;
    public static final double kScoreL2Angle = 75.0;
    public static final double kScoreL3Angle = 100.0;

    // --- TOLERANCE ---
    public static final double kAngleTolerance = 2.0;

    // --- TUNING ---
    public static final boolean kEnableAnglePIDTuning = true;
}

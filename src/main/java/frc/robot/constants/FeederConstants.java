package frc.robot.constants;

/**
 * Feeder alt sistemi için sabitler.
 * Intake → Shooter arası transfer.
 */
public final class FeederConstants {

    // --- MOTOR ID (RobotMap'ten) ---
    public static final int kFeederMotorID = RobotMap.kFeederMotorID;

    // --- PID CONFIG ---
    public static final double kP = 0.0001;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.00017;
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
    public static final double kRPMTolerance = 100.0; // Tolerans

    // --- SPEEDS (RPM) - Velocity kontrol için (encoder gerekli) ---
    public static final double kFeedRPM = 4500.0; // Shooter'a besleme hızı (~80% MAX)
    public static final double kSlowFeedRPM = 1000.0; // Yavaş besleme (Intake için)
    public static final double kIntakeRPM = 1500.0; // Intake'den alma hızı
    public static final double kReverseRPM = -1000.0; // Geri kusma hızı

    // --- SPEEDS (VOLTAGE) - Voltaj kontrol için (encoder gereksiz) ---
    public static final double kFeedVoltage = 8.0; // ~67% güç (12V max)
    public static final double kSlowFeedVoltage = 3.0; // ~25% güç
    public static final double kReverseVoltage = -4.0; // Geri kusma

    // --- CURRENT LIMIT ---
    public static final int kCurrentLimit = 30;
}

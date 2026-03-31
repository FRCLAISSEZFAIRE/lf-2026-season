package frc.robot.constants;

/**
 * Feeder alt sistemi için sabitler.
 * Indexer + Kicker ikili motor yapısı.
 * Intake → Shooter arası transfer.
 */
public final class FeederConstants {

    // --- MOTOR IDs (RobotMap'ten) ---
    public static final int kIndexerMotorID = RobotMap.kIndexerMotorID;
    public static final int kKickerMotorID = RobotMap.kKickerMotorID;

    // --- INDEXER PID CONFIG ---
    public static final double kIndexerP = 0.0006;
    public static final double kIndexerI = 0.0;
    public static final double kIndexerD = 0.0;
    public static final double kIndexerFF = 0.00018;

    // --- KICKER PID CONFIG ---
    public static final double kKickerP = 0.0006;
    public static final double kKickerI = 0.0;
    public static final double kKickerD = 0.0;
    public static final double kKickerFF = 0.00018;

    // --- PID OUTPUT RANGE ---
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
    public static final double kRPMTolerance = 100.0;

    // --- INDEXER SPEEDS (RPM) ---
    public static final double kIndexerFeedRPM = 5000.0;     // Shooter'a besleme hızı
    public static final double kIndexerSlowFeedRPM = 1000.0;  // Yavaş besleme
    public static final double kIndexerReverseRPM = -1000.0;  // Geri kusma hızı

    // --- KICKER SPEEDS (RPM) ---
    public static final double kKickerFeedRPM = 5000.0;      // Shooter'a fırlatma hızı
    public static final double kKickerSlowFeedRPM = 1000.0;   // Yavaş fırlatma
    public static final double kKickerReverseRPM = -1000.0;   // Geri kusma hızı

    // --- CURRENT LIMIT ---
    public static final int kCurrentLimit = 30;
}

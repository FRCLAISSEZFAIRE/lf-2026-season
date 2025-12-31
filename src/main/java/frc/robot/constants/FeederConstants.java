package frc.robot.constants;

/**
 * Feeder alt sistemi için sabitler.
 * Intake → Shooter arası transfer.
 */
public final class FeederConstants {

    // --- MOTOR ID (RobotMap'ten) ---
    public static final int kFeederMotorID = RobotMap.kFeederMotorID;

    // --- VOLTAGES ---
    public static final double kFeedVoltage = 8.0; // İleri besleme
    public static final double kReverseVoltage = -6.0; // Geri çıkarma

    // --- CURRENT LIMIT ---
    public static final int kCurrentLimit = 30;
}

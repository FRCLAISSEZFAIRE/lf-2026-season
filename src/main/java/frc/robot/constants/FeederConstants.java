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
    // --- CURRENT LIMIT ---
    public static final int kCurrentLimit = 30;

    // --- SENSORS (MZ-80 Digital Inputs) ---
    public static final int kFuelSensorBottomID = 5; // En alt seviye (Boş/Dolu)
    public static final int kFuelSensorLowID = 6;
    public static final int kFuelSensorHighID = 7;
    public static final int kFuelSensorTopID = 8; // En üst seviye (Full)
}

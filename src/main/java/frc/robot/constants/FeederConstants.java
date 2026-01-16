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
    public static final int kFuelSensorBottomID = RobotMap.kFeederSensorBottomID; // Giriş Sensörü (Hazne Boş değil)
    public static final int kFuelSensorTopID = RobotMap.kFeederSensorTopID;    // Çıkış Sensörü (Hazne Dolu/Namlu Ağzı)
}

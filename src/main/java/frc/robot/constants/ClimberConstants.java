package frc.robot.constants;

/**
 * Climber alt sistemi için sabitler.
 * İki Kraken motor ile voltaj kontrolü.
 */
public final class ClimberConstants {

    // --- MOTOR IDs (RobotMap'ten) ---
    public static final int kLeftMotorID = RobotMap.kClimberLeftMotorID;
    public static final int kRightMotorID = RobotMap.kClimberRightMotorID;

    // --- VOLTAGES ---
    public static final double kClimbUpVoltage = 12.0; // Tırmanma
    public static final double kClimbDownVoltage = -8.0; // İniş

    // --- CURRENT LIMITS ---
    public static final int kCurrentLimit = 60; // Kraken yüksek akım çekebilir
    public static final double kStallCurrentThreshold = 50.0; // Zorlanma eşiği

    // --- POSITION LIMITS (Rotor rotasyonu) ---
    public static final double kMaxPosition = 100.0;
    public static final double kMinPosition = 0.0;

    // --- SAFETY THRESHOLDS ---
    public static final double kTiltWarningThreshold = 15.0; // Derece - Uyarı
    public static final double kTiltDangerThreshold = 25.0; // Derece - Tehlike
}

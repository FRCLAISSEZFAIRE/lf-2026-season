package frc.robot.constants;

/**
 * Intake alt sistemi için sabitler.
 * Pivot(Extension) ve Roller motor ayarları.
 * Pivot artık rack-and-pinion sistemi ile SANTİMETRE (cm) cinsinden kontrol edilecek.
 */
public final class IntakeConstants {

    // ===========================================================================
    // EXTENSION GEAR RATIO & PINION DIAMETER
    // ===========================================================================

    /** Uzama (Extension) dişli oranı (motor devri / çıkış devri) */
    public static final double kExtensionGearRatio = 25.0;

    /** Kramayer dişli pinion mili çapı (cm) */
    public static final double kExtensionPinionDiameterCm = 5.0;

    // ===========================================================================
    // EXTENSION POSITIONS (çıkış santimetre)
    // ===========================================================================

    /** Intake tam açık (uzamış) pozisyonu (çıkış cm) - Maç boyunca bu konumda kalacak */
    public static final double kExtensionDeployedCm = 30.0;

    /** Intake tam kapalı (içeride) pozisyonu (çıkış cm) */
    public static final double kExtensionRetractedCm = 0.0;

    /** Uzama pozisyon toleransı (çıkış cm) */
    public static final double kExtensionToleranceCm = 1.0;

    // ===========================================================================
    // EXTENSION PID (Closed Loop - RIO'ya kaydedilir)
    // ===========================================================================

    public static final double kExtensionP = 0.02;
    public static final double kExtensionI = 0.0;
    public static final double kExtensionD = 0.001;

    public static final double kExtensionMaxVelocity = 120.0;
    public static final double kExtensionMaxAcceleration = 120.0;

    // ===========================================================================
    // EXTENSION LIMITS (çıkış santimetre - dashboard'dan ayarlanabilir)
    // ===========================================================================

    public static final double kExtensionMinCm = -2.0;
    public static final double kExtensionMaxCm = 40.0;

    // ===========================================================================
    // EXTENSION HOMING
    // ===========================================================================

    /** Homing voltajı (negatif = içeriye/geriye doğru çekme) */
    public static final double kExtensionHomingVoltage = -2.0;

    /** Homing süresi (saniye) */
    public static final double kExtensionHomingDurationSec = 1.0;

    // ===========================================================================
    // EXTENSION MOTOR
    // ===========================================================================

    public static final boolean kExtensionMotorInverted = false;
    public static final int kExtensionCurrentLimit = 40;

    // ===========================================================================
    // ROLLER (Kraken X60 - Velocity Control, RPM)
    // ===========================================================================

    public static final double kRollerVoltage = 8.0;
    public static final double kRollerkV = 0.12;
    public static final double kRollerkP = 0.11;
    public static final double kRollerkI = 0.0;
    public static final double kRollerkD = 0.0;

    public static final double kRollerTargetRPM = 4800.0;
    public static final double kRollerMinRPM = 0.0;
    public static final double kRollerMaxRPM = 6000.0;
    public static final boolean kRollerInverted = false;
    public static final int kRollerCurrentLimit = 60;

    // ===========================================================================
    // VISION / AUTO
    // ===========================================================================

    public static final double kAutoIntakeSpeed = 0.8;
    public static final double kTargetLostTimeoutSeconds = 4.0;
}

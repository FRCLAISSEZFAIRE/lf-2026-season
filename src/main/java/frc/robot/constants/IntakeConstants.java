package frc.robot.constants;

/**
 * Intake alt sistemi için sabitler.
 * Pivot ve Roller motor ayarları.
 * Tüm açılar DERECE cinsinden (çıkış açısı).
 */
public final class IntakeConstants {

    // ===========================================================================
    // PIVOT GEAR RATIO
    // ===========================================================================

    /** Pivot dişli oranı (motor devri / çıkış devri) */
    public static final double kPivotGearRatio = 25.0;

    // ===========================================================================
    // PIVOT POSITIONS (çıkış derece)
    // ===========================================================================

    /** Pivot açık pozisyonu (çıkış derece) - Maç boyunca bu konumda kalacak */
    public static final double kPivotDeployedDeg = 70.0;

    /** Pivot kapalı pozisyonu (çıkış derece) */
    public static final double kPivotRetractedDeg = 0.0;

    /** Pivot pozisyon toleransı (çıkış derece) */
    public static final double kPivotToleranceDeg = 5.0;

    // ===========================================================================
    // PIVOT PID (Closed Loop - RIO'ya kaydedilir)
    // ===========================================================================

    public static final double kPivotP = 0.02;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.001;

    public static final double kPivotMaxVelocity = 120.0;
    public static final double kPivotMaxAcceleration = 120.0;

    // ===========================================================================
    // PIVOT LIMITS (çıkış derece - dashboard'dan ayarlanabilir)
    // ===========================================================================

    public static final double kPivotMinDeg = -5.0;
    public static final double kPivotMaxDeg = 120.0;

    // ===========================================================================
    // PIVOT HOMING
    // ===========================================================================

    /** Homing voltajı (negatif = geriye doğru) */
    public static final double kPivotHomingVoltage = -2.0;

    /** Homing süresi (saniye) */
    public static final double kPivotHomingDurationSec = 1.0;

    // ===========================================================================
    // PIVOT MOTOR
    // ===========================================================================

    public static final boolean kPivotMotorInverted = false;
    public static final int kPivotCurrentLimit = 40;

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

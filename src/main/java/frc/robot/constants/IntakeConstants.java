package frc.robot.constants;

/**
 * Intake alt sistemi için sabitler.
 * Pivot ve Roller motor ayarları.
 */
public final class IntakeConstants {

    // ===========================================================================
    // PIVOT POSITIONS (radians)
    // ===========================================================================

    /** Pivot açık pozisyonu (radyan) */
    public static final double kPivotDeployedRad = 1.5;

    /** Pivot kapalı pozisyonu (radyan) */
    public static final double kPivotRetractedRad = 0.0;

    /** Pivot pozisyon toleransı (radyan) */
    public static final double kPivotToleranceRad = 0.1;

    // ===========================================================================
    // PIVOT PID
    // ===========================================================================

    /** Pivot P kazancı */
    public static final double kPivotP = 1.0;

    /** Pivot I kazancı */
    public static final double kPivotI = 0.0;

    /** Pivot D kazancı */
    public static final double kPivotD = 0.005;

    /** Pivot max velocity (rad/s) */
    public static final double kPivotMaxVelocity = 2.0;

    /** Pivot max acceleration (rad/s²) */
    public static final double kPivotMaxAcceleration = 2.0;

    // ===========================================================================
    // PIVOT LIMITS
    // ===========================================================================

    /** Pivot minimum açısı (radyan) */
    public static final double kPivotMinRad = -0.1;

    /** Pivot maximum açısı (radyan) */
    public static final double kPivotMaxRad = 2.0;

    // ===========================================================================
    // ROLLER
    // ===========================================================================

    // ===========================================================================
    // ROLLER (Kraken X60 - Velocity Control)
    // ===========================================================================

    /** Roller varsayılan voltajı (V) - Legacy */
    public static final double kRollerVoltage = 8.0;

    /** Roller Velocity Feedforward (kV) */
    public static final double kRollerkV = 0.12;
    /** Roller Velocity P Gain */
    public static final double kRollerkP = 0.11;
    /** Roller Velocity I Gain */
    public static final double kRollerkI = 0.0;
    /** Roller Velocity D Gain */
    public static final double kRollerkD = 0.0;

    /** Roller Hedef Hızı (RPS) */
    public static final double kRollerTargetRPS = 80.0; // ~4800 RPM

    /** Roller akım limiti (A) */
    public static final int kRollerCurrentLimit = 60;

    /** Pivot motor akım limiti (A) */
    public static final int kPivotCurrentLimit = 40;

    // ===========================================================================
    // VISION / AUTO
    // ===========================================================================

    /** Otomatik toplama hızı */
    public static final double kAutoIntakeSpeed = 0.8;

    /** Hedef kaybolursa kaç sn daha devam etsin */
    public static final double kTargetLostTimeoutSeconds = 4.0;
}

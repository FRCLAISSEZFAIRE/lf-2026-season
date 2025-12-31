package frc.robot.constants;

/**
 * Shooter alt sistemi için sabitler.
 * Flywheel, Turret ve Hood ayarları.
 */
public final class ShooterConstants {

    // --- HOOD (Atış Açısı) ---
    public static final double kHoodP = 0.05;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.002;

    public static final double kHoodMinAngle = 20.0; // Derece (düz atış)
    public static final double kHoodMaxAngle = 65.0; // Derece (dikey atış)
    public static final double kHoodTolerance = 1.0; // Derece

    // --- HOOD PRESETS ---
    public static final double kHoodCloseAngle = 55.0; // Yakın atış
    public static final double kHoodMidAngle = 40.0; // Orta mesafe
    public static final double kHoodFarAngle = 25.0; // Uzak atış

    // --- TURRET ---
    public static final double kTurretP = 4.0;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.1;
    public static final double kTurretTolerance = 0.05; // Radyan (~3 derece)

    // --- FLYWHEEL ---
    public static final double kFlywheelVoltage = 12.0;
    public static final double kFlywheelTargetRPM = 5000;
}

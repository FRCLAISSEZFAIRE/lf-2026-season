package frc.robot.constants;

/**
 * Shooter alt sistemi için sabitler.
 * Flywheel, Turret ve Hood ayarları.
 */
public final class ShooterConstants {

    // --- CONFIGURATION ---
    public static final boolean kIsTurreted = true; // True: Shooter dönebilir, False: Gövdeyle döner
    public static final boolean kHasDualFlywheels = false; // True: Çift flywheel (Sağ/Sol), False: Tek

    // --- HOOD (Servo) ---
    public static final int kHoodLeftPWMPort = 0;
    public static final int kHoodRightPWMPort = 1;
    
    public static final boolean kHoodLeftServoInverted = false;
    public static final boolean kHoodRightServoInverted = true;

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

    // --- FLYWHEEL (Kraken X60 - Velocity Control) ---
    public static final double kIdleFlywheelRPM = 2000.0; // Bekleme hızı
    public static final double kShootingFlywheelRPM = 6000.0; // Atış hızı
    public static final double kFlywheelToleranceRPM = 50.0;

    // Flywheel PID Gains (Slot 0)
    public static final double kFlywheelP = 0.2; // Tuning gerekli
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;
    public static final double kFlywheelkS = 0.25; // Static friction volt
    public static final double kFlywheelkV = 0.12; // Volt per (Rot/Sec) - Yaklaşık hesap
}

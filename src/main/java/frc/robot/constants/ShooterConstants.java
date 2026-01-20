package frc.robot.constants;

/**
 * Shooter alt sistemi için sabitler.
 * Flywheel, Turret ve Hood ayarları.
 * NOT: CAN ID'leri RobotMap'ten alınır.
 */
public final class ShooterConstants {

    // --- CONFIGURATION ---
    public static final boolean kIsTurreted = true;
    public static final boolean kHasDualFlywheels = false;
    public static final boolean kAutoAimEnabled = true;

    // ===========================================================================
    // TURRET (NEO Motor + Harici REV Absolute Encoder)
    // ===========================================================================

    // --- TURRET MEKANIK ---
    /** Motor:Turret dişli oranı. 1:20 = motor 20 tur döner, turret 1 tur döner */
    public static final double kTurretGearRatio = 20.0;

    /** Absolute encoder zero offset (derece). Calibrasyon için ayarlanır. */
    public static final double kTurretEncoderOffset = 0.0;

    // --- TURRET PID (Default değerler - TunableNumber ile override edilebilir) ---
    public static final double kTurretDefaultP = 0.1;
    public static final double kTurretDefaultI = 0.0;
    public static final double kTurretDefaultD = 0.005;

    /** Turret pozisyon toleransı (derece) */
    public static final double kTurretTolerance = 2.0;

    // --- TURRET SOFT LIMITS (Güvenlik - Derece) ---
    /** Turret minimum açısı (derece). Kablo sarma koruması. */
    public static final double kTurretMinAngle = -180.0;
    /** Turret maksimum açısı (derece). Kablo sarma koruması. */
    public static final double kTurretMaxAngle = 180.0;
    /** Soft limit aktif mi? */
    public static final boolean kTurretSoftLimitsEnabled = true;

    // --- TURRET CONTINUOUS WRAPPING ---
    /** 0-360 arası wrap için min input (derece) */
    public static final double kTurretWrapMinInput = -180.0;
    /** 0-360 arası wrap için max input (derece) */
    public static final double kTurretWrapMaxInput = 180.0;

    // ===========================================================================
    // HOOD (NEO 550 + SparkMax + Absolute Encoder)
    // ===========================================================================

    public static final double kHoodP = 2.0;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.05;

    public static final double kHoodMinAngle = 15.0;
    public static final double kHoodMaxAngle = 60.0;
    public static final double kHoodTolerance = 1.5;

    public static final double kHoodGearRatio = 1.0;
    public static final double kHoodEncoderOffset = 0.0;

    // Hood presets
    public static final double kHoodCloseAngle = 55.0;
    public static final double kHoodMidAngle = 40.0;
    public static final double kHoodFarAngle = 25.0;

    // ===========================================================================
    // FLYWHEEL (Kraken X60)
    // ===========================================================================

    public static final double kIdleFlywheelRPM = 2000.0;
    public static final double kFlywheelToleranceRPM = 100.0;

    public static final double kFlywheelP = 0.2;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;
    public static final double kFlywheelkS = 0.25;
    public static final double kFlywheelkV = 0.12;

    // ===========================================================================
    // SHOOTING PARAMETERS
    // ===========================================================================

    public static final double kCloseDistance = 1.5;
    public static final double kCloseHoodAngle = 55.0;
    public static final double kCloseFlywheelRPM = 4500.0;

    public static final double kFarDistance = 7.0;
    public static final double kFarHoodAngle = 17.0;
    public static final double kFarFlywheelRPM = 7000.0;

    public static final double kMinShootingDistance = kCloseDistance;
    public static final double kMaxShootingDistance = kFarDistance;

    // ===========================================================================
    // FEEDING MODE
    // ===========================================================================
    public static final double kFeedingFlywheelRPM = 3500.0;
    public static final double kFeedingHoodAngle = 50.0;

    // ===========================================================================
    // INTERPOLATION METHODS
    // ===========================================================================

    public static double getHoodAngleForDistance(double distanceMeters) {
        return interpolate(distanceMeters, kCloseDistance, kFarDistance, kCloseHoodAngle, kFarHoodAngle);
    }

    public static double getFlywheelRPMForDistance(double distanceMeters) {
        return interpolate(distanceMeters, kCloseDistance, kFarDistance, kCloseFlywheelRPM, kFarFlywheelRPM);
    }

    private static double interpolate(double input, double inputMin, double inputMax,
            double outputMin, double outputMax) {
        if (input <= inputMin)
            return outputMin;
        if (input >= inputMax)
            return outputMax;
        double t = (input - inputMin) / (inputMax - inputMin);
        return outputMin + t * (outputMax - outputMin);
    }
}

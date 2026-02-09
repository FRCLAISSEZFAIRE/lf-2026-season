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
    // TURRET (NEO Motor + Harici REV Through Bore Absolute Encoder)
    // ===========================================================================
    // Mekanik Yapı:
    // Motor → [1:4 Redüksiyon] → Encoder → [1:10 Dişli] → Turret
    // Motor 1 tur → Encoder 0.25 tur, Turret 0.025 tur
    // Motor 40 tur → Encoder 10 tur → Turret 1 tur
    // Encoder 1 tur → Turret 0.1 tur = 36 derece

    // --- TURRET MEKANIK ---
    /** Motor:Turret toplam dişli oranı (4 × 10 = 40) */
    public static final double kTurretGearRatio = 40.0;

    /** Encoder:Turret dişli oranı (encoder sonrası 1:10 dişli) */
    public static final double kTurretEncoderToTurretRatio = 10.0;

    /**
     * Absolute encoder'dan turret açısına dönüşüm:
     * 1 encoder tur = 1/10 turret tur = 36 derece
     * turretAngle (degrees) = encoderRotations × 36
     */
    public static final double kTurretEncoderMultiplier = 360.0 / kTurretEncoderToTurretRatio; // = 36 deg/encoder-rot

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
    public static final double kTurretMinAngle = -200.0;
    /** Turret maksimum açısı (derece). Kablo sarma koruması. */
    public static final double kTurretMaxAngle = 200.0;
    /** Soft limit aktif mi? */
    public static final boolean kTurretSoftLimitsEnabled = true;

    // --- TURRET CONTINUOUS WRAPPING ---
    /** 0-360 arası wrap için min input (derece) */
    public static final double kTurretWrapMinInput = -60.0;
    /** 0-360 arası wrap için max input (derece) */
    public static final double kTurretWrapMaxInput = 60.0;

    // ===========================================================================
    // HOOD (NEO 550 + SparkMax - Relative Encoder Only)
    // ===========================================================================
    // NOT: Hood'da absolute encoder YOK, relative encoder kullanılıyor.
    // Robot başlangıçta hood pozisyonunu bilmeli (home position).

    public static final double kHoodP = 2.0;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.05;

    public static final double kHoodMinAngle = 0.0;
    public static final double kHoodMaxAngle = 48.0;
    public static final double kHoodTolerance = 1.5;

    /** Hood dişli oranı. */
    public static final double kHoodGearRatio = 100.0;

    /** Hood başlangıç açısı (robot açılışında varsayılan pozisyon) */
    public static final double kHoodHomeAngle = 15.0;

    // Hood presets
    public static final double kHoodCloseAngle = 5.0;
    public static final double kHoodMidAngle = 15.0;
    public static final double kHoodFarAngle = 25.0;

    // ===========================================================================
    // FLYWHEEL (Kraken X60 - TEK MOTOR, FOLLOWER YOK)
    // ===========================================================================
    // NOT: Flywheel tek motor ile çalışıyor, ikinci motor/follower yok.

    /** Flywheel dişli oranı. */
    public static final double kFlywheelGearRatio = 1.0; // Direct drive

    public static final double kIdleFlywheelRPM = 2000.0;
    public static final double kFlywheelToleranceRPM = 100.0;
    public static final double kFlywheelTolerance = 200.0; // RPM tolerance for at-target check

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
    // DISTANCE-BASED CALIBRATION (Live Tunable in Test Mode)
    // ===========================================================================

    // --- HUB SHOOTING ---
    /**
     * Fixed distance points for Hub interpolation (meters).
     * UPDATED: {0.5, 1.0, 2.0, 3.5, 4.5}
     */
    public static final double[] HUB_DISTANCES = { 0.5, 1.0, 2.0, 3.5, 4.5 };

    /**
     * Default RPM values for Hub shooting.
     * Can be tuned via Dashboard in Test Mode.
     */
    public static final double[] DEFAULT_HUB_RPMS = { 2000, 2500, 3500, 4500, 5500 };

    /**
     * Default Hood angles for Hub shooting.
     * Can be tuned via Dashboard in Test Mode.
     */
    public static final double[] DEFAULT_HUB_HOOD_ANGLES = { 65.0, 60.0, 45.0, 30.0, 20.0 };

    // --- ALLIANCE PASS SHOOTING (4m - 8m) ---
    /**
     * Fixed distance points for Alliance Pass interpolation (meters).
     * Request: 4, 5, 6, 7, 8
     */
    public static final double[] ALLIANCE_PASS_DISTANCES = { 4.0, 5.0, 6.0, 7.0, 8.0 };

    /**
     * Default RPM values for Alliance Pass.
     * Placeholder values - Tuning Required.
     */
    public static final double[] DEFAULT_ALLIANCE_PASS_RPMS = { 3000, 3500, 4000, 4500, 5000 };

    /**
     * Default Hood angles for Alliance Pass.
     * Placeholder values - Tuning Required.
     */
    public static final double[] DEFAULT_ALLIANCE_PASS_HOOD_ANGLES = { 45.0, 40.0, 35.0, 30.0, 25.0 };

    // --- TOLERANCES ---
    public static final double SHOOTER_RPM_TOLERANCE = 50.0;
    public static final double HOOD_ANGLE_TOLERANCE = 1.0;
    public static final double TURRET_AIM_TOLERANCE = 2.0;

    /**
     * Default safe values for invalid/null distance (Fender shot)
     */
    public static final double FENDER_SHOT_RPM = 2000.0;
    public static final double FENDER_SHOT_HOOD_ANGLE = 65.0;

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

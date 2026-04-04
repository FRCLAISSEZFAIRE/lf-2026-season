package frc.robot.constants;

import frc.robot.util.TunableNumber;

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
        public static final double kTurretEncoderMultiplier = 360.0 / kTurretEncoderToTurretRatio; // = 36
                                                                                                   // deg/encoder-rot

        /** Absolute encoder zero offset (derece). Calibrasyon için ayarlanır. */
        public static final double kTurretEncoderOffset = 0.0;

        // --- TURRET PID (Default değerler - TunableNumber ile override edilebilir) ---
        public static final double kTurretDefaultP = 0.1;
        public static final double kTurretDefaultI = 0.0;
        public static final double kTurretDefaultD = 0.005;

        /**
         * Turret maksimum motor çıkışı (0.0 - 1.0 arası). Hız kontrolü için kullanılır.
         */
        public static final double kTurretMaxOutput = 0.2;

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

        /** Hood Smart Current Limit (Amps) - NEO 550 Safety */
        public static final int kHoodCurrentLimit = 20;

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
        // DISTANCE-BASED CALIBRATION (Tunable & Persistent)
        // ===========================================================================

        // --- HUB SHOOTING MAP ---
        // Point 0 (0.5m)
        public static final TunableNumber kHubDist0 = new TunableNumber("ShooterMap/Hub/Point0", "Dist", 0.5);
        public static final TunableNumber kHubRPM0 = new TunableNumber("ShooterMap/Hub/Point0", "RPM", 2000);
        public static final TunableNumber kHubHood0 = new TunableNumber("ShooterMap/Hub/Point0", "Hood", 0.0);

        // Point 1 (1.0m) - Measured
        public static final TunableNumber kHubDist1 = new TunableNumber("ShooterMap/Hub/Point1", "Dist", 1.0);
        public static final TunableNumber kHubRPM1 = new TunableNumber("ShooterMap/Hub/Point1", "RPM", 2650);
        public static final TunableNumber kHubHood1 = new TunableNumber("ShooterMap/Hub/Point1", "Hood", 0.0);

        // Point 2 (2.0m) - Measured
        public static final TunableNumber kHubDist2 = new TunableNumber("ShooterMap/Hub/Point2", "Dist", 2.0);
        public static final TunableNumber kHubRPM2 = new TunableNumber("ShooterMap/Hub/Point2", "RPM", 3000);
        public static final TunableNumber kHubHood2 = new TunableNumber("ShooterMap/Hub/Point2", "Hood", 15.0);

        // Point 3 (3.5m)
        public static final TunableNumber kHubDist3 = new TunableNumber("ShooterMap/Hub/Point3", "Dist", 3.5);
        public static final TunableNumber kHubRPM3 = new TunableNumber("ShooterMap/Hub/Point3", "RPM", 4500);
        public static final TunableNumber kHubHood3 = new TunableNumber("ShooterMap/Hub/Point3", "Hood", 30.0);

        // Point 4 (4.5m)
        public static final TunableNumber kHubDist4 = new TunableNumber("ShooterMap/Hub/Point4", "Dist", 4.5);
        public static final TunableNumber kHubRPM4 = new TunableNumber("ShooterMap/Hub/Point4", "RPM", 5500);
        public static final TunableNumber kHubHood4 = new TunableNumber("ShooterMap/Hub/Point4", "Hood", 45.0);

        // Arrays of Tunables for Iteration (Optional, but helpful for Subsystem)
        public static final TunableNumber[] HUB_DIST_TUNABLES = { kHubDist0, kHubDist1, kHubDist2, kHubDist3,
                        kHubDist4 };
        public static final TunableNumber[] HUB_RPM_TUNABLES = { kHubRPM0, kHubRPM1, kHubRPM2, kHubRPM3, kHubRPM4 };
        public static final TunableNumber[] HUB_HOOD_TUNABLES = { kHubHood0, kHubHood1, kHubHood2, kHubHood3,
                        kHubHood4 };

        // --- ALLIANCE PASS MAP ---
        // Point 0 (4.0m)
        public static final TunableNumber kPassDist0 = new TunableNumber("ShooterMap/Pass/Point0", "Dist", 4.0);
        public static final TunableNumber kPassRPM0 = new TunableNumber("ShooterMap/Pass/Point0", "RPM", 3000);
        public static final TunableNumber kPassHood0 = new TunableNumber("ShooterMap/Pass/Point0", "Hood", 50.0);

        // Point 1 (5.0m)
        public static final TunableNumber kPassDist1 = new TunableNumber("ShooterMap/Pass/Point1", "Dist", 5.0);
        public static final TunableNumber kPassRPM1 = new TunableNumber("ShooterMap/Pass/Point1", "RPM", 3500);
        public static final TunableNumber kPassHood1 = new TunableNumber("ShooterMap/Pass/Point1", "Hood", 50.0);

        // Point 2 (6.0m)
        public static final TunableNumber kPassDist2 = new TunableNumber("ShooterMap/Pass/Point2", "Dist", 6.0);
        public static final TunableNumber kPassRPM2 = new TunableNumber("ShooterMap/Pass/Point2", "RPM", 4000);
        public static final TunableNumber kPassHood2 = new TunableNumber("ShooterMap/Pass/Point2", "Hood", 50.0);

        // Point 3 (7.0m)
        public static final TunableNumber kPassDist3 = new TunableNumber("ShooterMap/Pass/Point3", "Dist", 7.0);
        public static final TunableNumber kPassRPM3 = new TunableNumber("ShooterMap/Pass/Point3", "RPM", 4500);
        public static final TunableNumber kPassHood3 = new TunableNumber("ShooterMap/Pass/Point3", "Hood", 50.0);

        // Point 4 (8.0m)
        public static final TunableNumber kPassDist4 = new TunableNumber("ShooterMap/Pass/Point4", "Dist", 8.0);
        public static final TunableNumber kPassRPM4 = new TunableNumber("ShooterMap/Pass/Point4", "RPM", 5000);
        public static final TunableNumber kPassHood4 = new TunableNumber("ShooterMap/Pass/Point4", "Hood", 50.0);

        public static final TunableNumber[] PASS_DIST_TUNABLES = { kPassDist0, kPassDist1, kPassDist2, kPassDist3,
                        kPassDist4 };
        public static final TunableNumber[] PASS_RPM_TUNABLES = { kPassRPM0, kPassRPM1, kPassRPM2, kPassRPM3,
                        kPassRPM4 };
        public static final TunableNumber[] PASS_HOOD_TUNABLES = { kPassHood0, kPassHood1, kPassHood2, kPassHood3,
                        kPassHood4 };

        // --- TOLERANCES (Dashboard Tunable) ---
        public static final TunableNumber SHOOTER_RPM_TOLERANCE = new TunableNumber("Shooter/Tolerances", "RPM", 50.0);
        public static final TunableNumber HOOD_ANGLE_TOLERANCE = new TunableNumber("Shooter/Tolerances", "HoodAngle", 2.0);
        public static final TunableNumber TURRET_AIM_TOLERANCE = new TunableNumber("Shooter/Tolerances", "TurretAim", 2.0);

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
        // INTERPOLATION METHODS (REMOVED - MOVED TO SUBSYSTEM)
        // ===========================================================================
        // The static interpolate method is generic and can stay if needed,
        // but specific getHoodAngleForDistance etc. must removed as they relied on
        // arrays.

        public static double interpolate(double input, double inputMin, double inputMax,
                        double outputMin, double outputMax) {
                if (input <= inputMin)
                        return outputMin;
                if (input >= inputMax)
                        return outputMax;
                double t = (input - inputMin) / (inputMax - inputMin);
                return outputMin + t * (outputMax - outputMin);
        }

        // ===========================================================================
        // FIXED SHOOTING POSITIONS (Sabit Atış Noktaları)
        // ===========================================================================
        // Her nokta için: X, Y (konum referansı), TurretAngle (derece), RPM, HoodAngle
        // (derece)
        // Tüm değerler TunableNumber — Dashboard'dan ayarlanabilir ve RoboRIO'da
        // kalıcı.
        // Field verisi KULLANILMAZ — tamamen sabit değerler.

        /** Sabit atış noktası isimleri (Red) */
        public static final String[] RED_FIXED_SHOT_NAMES = { "R1", "R2", "R3", "R4", "RP1", "RP2" };
        /** Sabit atış noktası isimleri (Blue) */
        public static final String[] BLUE_FIXED_SHOT_NAMES = { "B1", "B2", "B3", "B4", "BP1", "BP2" };

        /** Toplam nokta sayısı (her alliance için) */
        public static final int FIXED_SHOT_COUNT = 6;

        // --- RED ALLIANCE FIXED SHOTS ---

        // R1
        public static final TunableNumber kR1_X = new TunableNumber("FixedShot/R1", "X", 13.0);
        public static final TunableNumber kR1_Y = new TunableNumber("FixedShot/R1", "Y", 0.6);
        public static final TunableNumber kR1_TurretAngle = new TunableNumber("FixedShot/R1", "TurretAngle", 79.5);
        public static final TunableNumber kR1_RPM = new TunableNumber("FixedShot/R1", "RPM", 4300);
        public static final TunableNumber kR1_HoodAngle = new TunableNumber("FixedShot/R1", "HoodAngle", 29);

        // R2
        public static final TunableNumber kR2_X = new TunableNumber("FixedShot/R2", "X", 13.0);
        public static final TunableNumber kR2_Y = new TunableNumber("FixedShot/R2", "Y", 2.4);
        public static final TunableNumber kR2_TurretAngle = new TunableNumber("FixedShot/R2", "TurretAngle", 68.3);
        public static final TunableNumber kR2_RPM = new TunableNumber("FixedShot/R2", "RPM", 2890);
        public static final TunableNumber kR2_HoodAngle = new TunableNumber("FixedShot/R2", "HoodAngle", 10.0);

        // R3
        public static final TunableNumber kR3_X = new TunableNumber("FixedShot/R3", "X", 13.0);
        public static final TunableNumber kR3_Y = new TunableNumber("FixedShot/R3", "Y", 5.5);
        public static final TunableNumber kR3_TurretAngle = new TunableNumber("FixedShot/R3", "TurretAngle", -67.0);
        public static final TunableNumber kR3_RPM = new TunableNumber("FixedShot/R3", "RPM", 2860.0);
        public static final TunableNumber kR3_HoodAngle = new TunableNumber("FixedShot/R3", "HoodAngle", 9.0);

        // R4
        public static final TunableNumber kR4_X = new TunableNumber("FixedShot/R4", "X", 13.0);
        public static final TunableNumber kR4_Y = new TunableNumber("FixedShot/R4", "Y", 7.4);
        public static final TunableNumber kR4_TurretAngle = new TunableNumber("FixedShot/R4", "TurretAngle", -79.5);
        public static final TunableNumber kR4_RPM = new TunableNumber("FixedShot/R4", "RPM", 4417.0);
        public static final TunableNumber kR4_HoodAngle = new TunableNumber("FixedShot/R4", "HoodAngle", 29.1);

        // RP1 (Red Pass 1)
        public static final TunableNumber kRP1_X = new TunableNumber("FixedShot/RP1", "X", 10.8);
        public static final TunableNumber kRP1_Y = new TunableNumber("FixedShot/RP1", "Y", 5.6);
        public static final TunableNumber kRP1_TurretAngle = new TunableNumber("FixedShot/RP1", "TurretAngle", -162);
        public static final TunableNumber kRP1_RPM = new TunableNumber("FixedShot/RP1", "RPM", 3000.0);
        public static final TunableNumber kRP1_HoodAngle = new TunableNumber("FixedShot/RP1", "HoodAngle", 45.0);

        // RP2 (Red Pass 2)
        public static final TunableNumber kRP2_X = new TunableNumber("FixedShot/RP2", "X", 10.8);
        public static final TunableNumber kRP2_Y = new TunableNumber("FixedShot/RP2", "Y", 2.3);
        public static final TunableNumber kRP2_TurretAngle = new TunableNumber("FixedShot/RP2", "TurretAngle", 162.0);
        public static final TunableNumber kRP2_RPM = new TunableNumber("FixedShot/RP2", "RPM", 3000.0);
        public static final TunableNumber kRP2_HoodAngle = new TunableNumber("FixedShot/RP2", "HoodAngle", 45.0);

        // --- BLUE ALLIANCE FIXED SHOTS ---

        // B1 (Mirror of R1: X=16.49-13.0=3.49, Y=8.10-0.6=7.5, Turret=-79.5)
        public static final TunableNumber kB1_X = new TunableNumber("FixedShot/B1", "X", 3.49);
        public static final TunableNumber kB1_Y = new TunableNumber("FixedShot/B1", "Y", 7.5);
        public static final TunableNumber kB1_TurretAngle = new TunableNumber("FixedShot/B1", "TurretAngle", -79.5);
        public static final TunableNumber kB1_RPM = new TunableNumber("FixedShot/B1", "RPM", 4300);
        public static final TunableNumber kB1_HoodAngle = new TunableNumber("FixedShot/B1", "HoodAngle", 29);

        // B2 (Mirror of R2: X=16.49-13.0=3.49, Y=8.10-2.4=5.7, Turret=-68.3)
        public static final TunableNumber kB2_X = new TunableNumber("FixedShot/B2", "X", 3.49);
        public static final TunableNumber kB2_Y = new TunableNumber("FixedShot/B2", "Y", 5.7);
        public static final TunableNumber kB2_TurretAngle = new TunableNumber("FixedShot/B2", "TurretAngle", -68.3);
        public static final TunableNumber kB2_RPM = new TunableNumber("FixedShot/B2", "RPM", 2890);
        public static final TunableNumber kB2_HoodAngle = new TunableNumber("FixedShot/B2", "HoodAngle", 10.0);

        // B3 (Mirror of R3: X=16.49-13.0=3.49, Y=8.10-5.5=2.6, Turret=67.0)
        public static final TunableNumber kB3_X = new TunableNumber("FixedShot/B3", "X", 3.49);
        public static final TunableNumber kB3_Y = new TunableNumber("FixedShot/B3", "Y", 2.6);
        public static final TunableNumber kB3_TurretAngle = new TunableNumber("FixedShot/B3", "TurretAngle", 67.0);
        public static final TunableNumber kB3_RPM = new TunableNumber("FixedShot/B3", "RPM", 2860.0);
        public static final TunableNumber kB3_HoodAngle = new TunableNumber("FixedShot/B3", "HoodAngle", 9.0);

        // B4 (Mirror of R4: X=16.49-13.0=3.49, Y=8.10-7.4=0.7, Turret=79.5)
        public static final TunableNumber kB4_X = new TunableNumber("FixedShot/B4", "X", 3.49);
        public static final TunableNumber kB4_Y = new TunableNumber("FixedShot/B4", "Y", 0.7);
        public static final TunableNumber kB4_TurretAngle = new TunableNumber("FixedShot/B4", "TurretAngle", 79.5);
        public static final TunableNumber kB4_RPM = new TunableNumber("FixedShot/B4", "RPM", 4417.0);
        public static final TunableNumber kB4_HoodAngle = new TunableNumber("FixedShot/B4", "HoodAngle", 29.1);

        // BP1 (Mirror of RP1: X=16.49-10.8=5.69, Y=8.10-5.6=2.5, Turret=162)
        public static final TunableNumber kBP1_X = new TunableNumber("FixedShot/BP1", "X", 5.69);
        public static final TunableNumber kBP1_Y = new TunableNumber("FixedShot/BP1", "Y", 2.5);
        public static final TunableNumber kBP1_TurretAngle = new TunableNumber("FixedShot/BP1", "TurretAngle", 162.0);
        public static final TunableNumber kBP1_RPM = new TunableNumber("FixedShot/BP1", "RPM", 3000.0);
        public static final TunableNumber kBP1_HoodAngle = new TunableNumber("FixedShot/BP1", "HoodAngle", 45.0);

        // BP2 (Mirror of RP2: X=16.49-10.8=5.69, Y=8.10-2.3=5.8, Turret=-162)
        public static final TunableNumber kBP2_X = new TunableNumber("FixedShot/BP2", "X", 5.69);
        public static final TunableNumber kBP2_Y = new TunableNumber("FixedShot/BP2", "Y", 5.8);
        public static final TunableNumber kBP2_TurretAngle = new TunableNumber("FixedShot/BP2", "TurretAngle", -162.0);
        public static final TunableNumber kBP2_RPM = new TunableNumber("FixedShot/BP2", "RPM", 3000.0);
        public static final TunableNumber kBP2_HoodAngle = new TunableNumber("FixedShot/BP2", "HoodAngle", 45.0);

        // --- FIXED SHOT ARRAYS (Kolay erişim için) ---
        // Sıralama: [0]=R1/B1, [1]=R2/B2, [2]=R3/B3, [3]=R4/B4, [4]=RP1/BP1,
        // [5]=RP2/BP2

        public static final TunableNumber[] RED_FIXED_TURRET = {
                        kR1_TurretAngle, kR2_TurretAngle, kR3_TurretAngle, kR4_TurretAngle,
                        kRP1_TurretAngle, kRP2_TurretAngle };

        public static final TunableNumber[] RED_FIXED_RPM = {
                        kR1_RPM, kR2_RPM, kR3_RPM, kR4_RPM,
                        kRP1_RPM, kRP2_RPM };

        public static final TunableNumber[] RED_FIXED_HOOD = {
                        kR1_HoodAngle, kR2_HoodAngle, kR3_HoodAngle, kR4_HoodAngle,
                        kRP1_HoodAngle, kRP2_HoodAngle };

        public static final TunableNumber[] BLUE_FIXED_TURRET = {
                        kB1_TurretAngle, kB2_TurretAngle, kB3_TurretAngle, kB4_TurretAngle,
                        kBP1_TurretAngle, kBP2_TurretAngle };

        public static final TunableNumber[] BLUE_FIXED_RPM = {
                        kB1_RPM, kB2_RPM, kB3_RPM, kB4_RPM,
                        kBP1_RPM, kBP2_RPM };

        public static final TunableNumber[] BLUE_FIXED_HOOD = {
                        kB1_HoodAngle, kB2_HoodAngle, kB3_HoodAngle, kB4_HoodAngle,
                        kBP1_HoodAngle, kBP2_HoodAngle };
}

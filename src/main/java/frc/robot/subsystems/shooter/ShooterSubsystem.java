package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMap;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.Supplier;

/**
 * REVLib 2026 tabanlı Shooter alt sistemi.
 * setSetpoint API kullanılarak yapılandırılmış.
 * 
 * <h2>Bileşenler:</h2>
 * <ul>
 * <li>Turret: NEO + Relative Encoder (REVLib Onboard PID, derece)</li>
 * <li>Hood: NEO + Relative Encoder (REVLib Onboard PID, derece)</li>
 * <li>Flywheel: Kraken X60 (Phoenix6 Velocity, RPM)</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

    private final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");

    public enum ShooterMode {
        IDLE,
        SCORING,
        FEEDING
    }

    // =====================================================================
    // MOTOR CONTROLLERS
    // =====================================================================
    private final SparkMax turretMotor;
    private final SparkMax hoodMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX flywheelFollowerMotor;

    // Flywheel: Phoenix6 velocity control
    private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0);

    // =====================================================================
    // STATE VARIABLES
    // =====================================================================
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    private ShooterMode currentMode = ShooterMode.IDLE;
    private boolean isAutoAimActive = false;

    // Target values (HEPSİ DERECE veya RPM)
    private double turretTargetDeg = 0;
    private double hoodTargetDeg = ShooterConstants.kHoodHomeAngle;
    private double flywheelTargetRPM = 0; // Varsayılan: KAPALI. Sadece ShootCommand çalıştırır.
                                          // Default: OFF. Only ShootCommand activates flywheel.
    private double autoAimOffsetDeg = 0.0;
    private double hubOffsetX = 0.0; // Hub X offset (metre)
    private double hubOffsetY = 0.0; // Hub Y offset (metre)

    // EMA-smoothed velocity for moving shoot (prevents sudden jumps)
    private double smoothedVx = 0.0;
    private double smoothedVy = 0.0;

    // Hood hysteresis — only update hood when distance changes significantly
    private double lastHoodUpdateDistanceM = -1.0;
    private static final double HOOD_UPDATE_THRESHOLD_M = 0.15;

    // =====================================================================
    // CALIBRATION MAPS
    // =====================================================================
    private final InterpolatingDoubleTreeMap hoodCalibrationMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelCalibrationMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAlliancePassMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelAlliancePassMap = new InterpolatingDoubleTreeMap();

    // =====================================================================
    // TUNABLE NUMBERS (Dashboard Control)
    // =====================================================================
    // Turret PID
    private final TunableNumber turretKP = new TunableNumber("Shooter", "Turret kP", ShooterConstants.kTurretDefaultP);
    private final TunableNumber turretKI = new TunableNumber("Shooter", "Turret kI", ShooterConstants.kTurretDefaultI);
    private final TunableNumber turretKD = new TunableNumber("Shooter", "Turret kD", ShooterConstants.kTurretDefaultD);

    // Hood PID
    private final TunableNumber hoodKP = new TunableNumber("Shooter", "Hood kP", ShooterConstants.kHoodP);
    private final TunableNumber hoodKI = new TunableNumber("Shooter", "Hood kI", ShooterConstants.kHoodI);
    private final TunableNumber hoodKD = new TunableNumber("Shooter", "Hood kD", ShooterConstants.kHoodD);

    // Flywheel PID
    private final TunableNumber flywheelKP = new TunableNumber("Shooter", "Flywheel kP", ShooterConstants.kFlywheelP);
    private final TunableNumber flywheelKI = new TunableNumber("Shooter", "Flywheel kI", ShooterConstants.kFlywheelI);
    private final TunableNumber flywheelKD = new TunableNumber("Shooter", "Flywheel kD", ShooterConstants.kFlywheelD);
    private final TunableNumber flywheelKV = new TunableNumber("Shooter", "Flywheel kV", ShooterConstants.kFlywheelkV);
    private final TunableNumber flywheelTolerance = new TunableNumber("Shooter", "Flywheel Tolerance",
            ShooterConstants.kFlywheelTolerance);

    // Turret Max Output (Hız kontrolü - Dashboard'dan ayarlanabilir)
    private final TunableNumber turretMaxOutput = new TunableNumber("Shooter", "Turret MaxOutput",
            ShooterConstants.kTurretMaxOutput);

    // Turret Center Offset (Mekanik merkez kayması - Dashboard'dan ayarlanabilir)
    // Encoder 0° pozisyonu ile gerçek mekanik merkez arasındaki fark (derece)
    private final TunableNumber turretCenterOffset = new TunableNumber("Shooter", "Turret Center", 0.0);

    // Turret Soft Limits (Dashboard'dan ayarlanabilir)
    private final TunableNumber turretMinAngle = new TunableNumber("Shooter", "Turret MinAngle",
            ShooterConstants.kTurretMinAngle);
    private final TunableNumber turretMaxAngle = new TunableNumber("Shooter", "Turret MaxAngle",
            ShooterConstants.kTurretMaxAngle);

    // Hood Soft Limits (Dashboard'dan ayarlanabilir)
    private final TunableNumber hoodMaxAngle = new TunableNumber("Shooter", "Hood MaxAngle",
            ShooterConstants.kHoodMaxAngle);

    // Turret Gear Ratios (Tunable for calibration)
    private final TunableNumber turretGearRatio = new TunableNumber("Shooter", "Turret Gear Ratio",
            ShooterConstants.kTurretGearRatio);
    private final TunableNumber turretEncoderToTurretRatio = new TunableNumber("Shooter",
            "Enc to Turret Ratio", ShooterConstants.kTurretEncoderToTurretRatio);

    // Hood Gear Ratio
    private final TunableNumber hoodGearRatio = new TunableNumber("Shooter", "Hood Gear Ratio",
            ShooterConstants.kHoodGearRatio);

    // Turret Physical Position (Robot frame — robot merkezine göre taret pozisyonu,
    private final TunableNumber turretPositionX = new TunableNumber("Shooter", "Turret Center X", 15.0);
    private final TunableNumber turretPositionY = new TunableNumber("Shooter", "Turret Center Y", 15.0);

    // =====================================================================
    // MOVING SHOOT TUNABLES
    // Hareket halinde atış fizik parametreleri — Dashboard'dan ayarlanabilir
    // Moving shoot physics parameters — adjustable from Dashboard
    // =====================================================================
    private final frc.robot.util.TunableBoolean enableMovingShoot = new frc.robot.util.TunableBoolean("Shooter",
            "Enable Moving Shoot", true);
    private final frc.robot.util.TunableBoolean followerInvert = new frc.robot.util.TunableBoolean("Shooter",
            "Follower Invert", true);
    private final TunableNumber averageShotVelocityMps = new TunableNumber("Shooter", "Average Shot Velocity m/s",
            18.0);

    // Mekanik gecikme: feeder tetiklemesinden topun namluyu terk etmesine kadar
    // geçen süre (ms)
    // Mechanical latency: time from feeder trigger to ball exiting barrel (ms)
    private final TunableNumber mechanicalLatencyMs = new TunableNumber("Shooter", "Mech Latency ms", 80.0);

    // Taret PID tepki süresi: öncüleme açısı hesabında kullanılır (ms)
    // Turret PID response time: used in lead angle calculation (ms)
    private final TunableNumber turretResponseMs = new TunableNumber("Shooter", "Turret Response ms", 100.0);

    // Gecikme telafisi aktif/pasif
    // Latency compensation enable/disable
    private final frc.robot.util.TunableBoolean enableLatencyComp = new frc.robot.util.TunableBoolean("Shooter",
            "Enable Latency Comp", true);

    // Taret öncüleme açısı aktif/pasif
    // Turret lead angle enable/disable
    private final frc.robot.util.TunableBoolean enableTurretLead = new frc.robot.util.TunableBoolean("Shooter",
            "Enable Turret Lead", true);

    // Hız kompanzasyon kazanç çarpanı (P kazanç gibi kullanılır)
    // Velocity compensation gain multiplier (used like a P gain)
    // 1.0 = fizik doğru / 0.0 = kompanzasyon kapalı / >1.0 = agresif
    // 1.0 = physics-correct / 0.0 = compensation off / >1.0 = aggressive
    private final TunableNumber movingShootVelGain = new TunableNumber("Shooter", "Moving Shoot Vel Gain", 1.0);

    // EMA alpha for velocity smoothing: 0.0=frozen, 1.0=raw (no filter)
    // 0.2 @ 50Hz → ~80ms time constant
    private final TunableNumber velocitySmoothAlpha = new TunableNumber("Shooter", "Velocity Smooth Alpha", 0.2);

    // =====================================================================
    // CONSTRUCTOR
    // =====================================================================
    public ShooterSubsystem(Supplier<Pose2d> robotPoseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        // --- TURRET MOTOR (NEO + Relative Encoder + Onboard PID) ---
        turretMotor = new SparkMax(RobotMap.kTurretMotorID, MotorType.kBrushless);
        configureTurretMotor();

        // --- HOOD MOTOR (NEO + Relative Encoder + Onboard PID) ---
        hoodMotor = new SparkMax(RobotMap.kHoodMotorID, MotorType.kBrushless);
        configureHoodMotor();

        // --- FLYWHEEL MOTOR (Kraken X60) ---
        flywheelMotor = new TalonFX(RobotMap.kShooterMasterID);
        flywheelFollowerMotor = new TalonFX(RobotMap.kShooterFollowerID);
        configureFlywheelMotor();

        System.out.println("[Shooter] REVLib 2026 setSetpoint API ile yapılandırıldı");

        // --- CALIBRATION INITIALIZATION ---
        initializeCalibration();
    }

    // =====================================================================
    // MOTOR CONFIGURATION OBJECTS (Stored for runtime updates)
    // =====================================================================
    private final SparkMaxConfig turretConfig = new SparkMaxConfig();
    private final SparkMaxConfig hoodConfig = new SparkMaxConfig();
    private final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    // =====================================================================
    // CALIBRATION INITIALIZATION & UPDATES
    // =====================================================================

    /**
     * Rebuilds the interpolation maps from the TunableNumber values in
     * ShooterConstants.
     * Called during initialization and periodically in Test Mode via
     * updateCalibrationMapsFromDashboard.
     */
    private void createShooterMaps() {
        // Clear existing maps
        hoodCalibrationMap.clear();
        flywheelCalibrationMap.clear();
        hoodAlliancePassMap.clear();
        flywheelAlliancePassMap.clear();

        // --- HUB MAPS ---
        for (int i = 0; i < ShooterConstants.HUB_DIST_TUNABLES.length; i++) {
            double dist = ShooterConstants.HUB_DIST_TUNABLES[i].get();
            double rpm = ShooterConstants.HUB_RPM_TUNABLES[i].get();
            double hood = ShooterConstants.HUB_HOOD_TUNABLES[i].get();

            hoodCalibrationMap.put(dist, hood);
            flywheelCalibrationMap.put(dist, rpm);
        }

        // --- ALLIANCE PASS MAPS ---
        for (int i = 0; i < ShooterConstants.PASS_DIST_TUNABLES.length; i++) {
            double dist = ShooterConstants.PASS_DIST_TUNABLES[i].get();
            double rpm = ShooterConstants.PASS_RPM_TUNABLES[i].get();
            double hood = ShooterConstants.PASS_HOOD_TUNABLES[i].get();

            hoodAlliancePassMap.put(dist, hood);
            flywheelAlliancePassMap.put(dist, rpm);
        }
    }

    private void initializeCalibration() {
        createShooterMaps();
        System.out.println("[Shooter] Calibration maps initialized from TunableNumbers");
    }

    /**
     * Checks if any TunableNumber has changed and rebuilds maps if necessary.
     * Called only in Test Mode to save performance.
     */
    private void updateCalibrationMapsFromDashboard() {
        boolean hasChanged = false;

        // Check Hub Tunables
        for (TunableNumber t : ShooterConstants.HUB_DIST_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.HUB_RPM_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.HUB_HOOD_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;

        // Check Pass Tunables
        for (TunableNumber t : ShooterConstants.PASS_DIST_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.PASS_RPM_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;
        for (TunableNumber t : ShooterConstants.PASS_HOOD_TUNABLES)
            if (t.hasChanged())
                hasChanged = true;

        if (hasChanged) {
            createShooterMaps();
            System.out.println("[Shooter] Interpolation Maps Updated from Dashboard!");
        }
    }

    // =====================================================================
    // SHOOTER STATE CALCULATION (Distance-Based Lookup)
    // =====================================================================

    /**
     * Represents the target state for the shooter at a given distance.
     */
    public static class ShooterState {
        public final double rpm;
        public final double hoodAngleDeg;

        public ShooterState(double rpm, double hoodAngleDeg) {
            this.rpm = rpm;
            this.hoodAngleDeg = hoodAngleDeg;
        }
    }

    /**
     * Calculate the shooter state (RPM and Hood Angle) for a given distance.
     * Uses InterpolatingDoubleTreeMap for smooth interpolation between calibration
     * points.
     * 
     * @param distanceMeters Distance to target in meters. If null/invalid, defaults
     *                       to fender shot.
     * @return ShooterState containing target RPM and hood angle
     */
    public ShooterState calculateShooterState(double distanceMeters) {
        // Safety check: clamp distance to valid range
        double minDistance = ShooterConstants.kHubDist0.get();
        double maxDistance = ShooterConstants.kHubDist4.get();

        double clampedDistance = MathUtil.clamp(distanceMeters, minDistance, maxDistance);

        // Invalid/NaN distance check - default to fender shot
        if (Double.isNaN(distanceMeters) || distanceMeters <= 0) {
            return new ShooterState(
                    ShooterConstants.FENDER_SHOT_RPM,
                    ShooterConstants.FENDER_SHOT_HOOD_ANGLE);
        }

        // Lookup from interpolation maps
        double rpm = flywheelCalibrationMap.get(clampedDistance);
        double hoodAngle = hoodCalibrationMap.get(clampedDistance);

        // Log for debugging
        Logger.recordOutput("Tuning/Shooter/Calculated/Distance", distanceMeters);
        Logger.recordOutput("Tuning/Shooter/Calculated/RPM", rpm);
        Logger.recordOutput("Tuning/Shooter/Calculated/HoodAngle", hoodAngle);

        return new ShooterState(rpm, hoodAngle);
    }

    /**
     * Calculates the ideal shooter state for ALLIANCE PASS (To Alliance Wall).
     * Distances: 4m - 8m
     */
    public ShooterState calculateShooterStateForAlliancePass(double distanceMeters) {
        // Safety check: clamp distance to valid range
        double minDistance = ShooterConstants.kPassDist0.get();
        double maxDistance = ShooterConstants.kPassDist4.get();

        double clampedDistance = MathUtil.clamp(distanceMeters, minDistance, maxDistance);

        double targetRPM = flywheelAlliancePassMap.get(clampedDistance);
        double targetHood = hoodAlliancePassMap.get(clampedDistance);

        return new ShooterState(targetRPM, targetHood);
    }

    /**
     * Apply shooter state - sets both flywheel RPM and hood angle.
     * 
     * @param state ShooterState to apply
     */
    public void applyShooterState(ShooterState state) {
        setFlywheelRPM(state.rpm + flywheelOffsetRPM);
        setHoodAngle(state.hoodAngleDeg + hoodOffsetDeg);
    }

    // =====================================================================
    // MOTOR CONFIGURATION
    // =====================================================================
    private void configureTurretMotor() {
        // Relative encoder yapılandırması (dahili encoder)
        // Motor:Turret oranı 40:1 → 1 motor tur = 9 derece turret
        double currentRatio = turretGearRatio.get();
        double degreesPerMotorRot = 360.0 / currentRatio;
        turretConfig.encoder
                .positionConversionFactor(degreesPerMotorRot)
                .velocityConversionFactor(degreesPerMotorRot / 60.0);

        // REVLib onboard PID (setSetpoint ile kullanılacak)
        turretConfig.closedLoop
                .p(turretKP.get())
                .i(turretKI.get())
                .d(turretKD.get())
                .outputRange(-turretMaxOutput.get(), turretMaxOutput.get()); // Dashboard'dan ayarlanabilir hız

        // Motor ayarları
        turretConfig.inverted(true);
        turretConfig.idleMode(IdleMode.kBrake);

        // Soft limits (derece olarak)
        turretConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit((float) turretMaxAngle.get())
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit((float) turretMinAngle.get());

        // Magnet / Limit Switch Yapılandırması (REVLib 2026)
        // Taret 0 noktasından (-200'e kadar) dönebildiği için limit switch'in motoru
        // donanımsal olarak durdurmasını kapatıyoruz (false). Sadece homing için
        // okuyacağız.
        turretConfig.limitSwitch
                .reverseLimitSwitchEnabled(false)
                .reverseLimitSwitchType(com.revrobotics.spark.config.LimitSwitchConfig.Type.kNormallyOpen);

        // CAN Optimization: Turret Motor
        turretConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                // We use position control so keep position at default (20ms), slow down
                // velocity
                .primaryEncoderVelocityPeriodMs(500);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder'ı sıfırla - TARET 0° POZİSYONUNDA BAŞLATILMALI!
        turretMotor.getEncoder().setPosition(0);

        System.out.println("[Shooter] Turret: Onboard PID, Relative Encoder, Soft Limits ±180°");
        System.out.println("[Shooter] ⚠️ TARET 0° POZİSYONUNDA BAŞLATILMALI!");
    }

    private void configureHoodMotor() {
        // Relative encoder dönüşümü (motor rotasyonu → derece)
        double currentRatio = hoodGearRatio.get();
        double degreesPerMotorRot = 360.0 / currentRatio;
        hoodConfig.encoder
                .positionConversionFactor(degreesPerMotorRot)
                .velocityConversionFactor(degreesPerMotorRot / 60.0);

        // REVLib onboard PID
        hoodConfig.closedLoop
                .p(hoodKP.get())
                .i(hoodKI.get())
                .d(hoodKD.get())
                .outputRange(-0.5, 0.5); // Güvenlik limiti artırıldı (0.2 -> 0.5)

        // Motor ayarları
        hoodConfig.inverted(true);
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.smartCurrentLimit(ShooterConstants.kHoodCurrentLimit);

        // Soft Limits Ekleme
        hoodConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit((float) hoodMaxAngle.get())
                .reverseSoftLimitEnabled(true)
                // Homing sırasında soft limit kapatılıp CAN'i bloklamasın diye 2 derece
                // güvenlik payı bırakıldı
                .reverseSoftLimit((float) (ShooterConstants.kHoodMinAngle - 2.0));

        // CAN Optimization: Hood Motor
        hoodConfig.signals
                .absoluteEncoderPositionPeriodMs(500)
                .absoluteEncoderVelocityPeriodMs(500)
                .analogVoltagePeriodMs(500)
                // We use position control so keep position at default (20ms), slow down
                // velocity
                .primaryEncoderVelocityPeriodMs(500);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder'ı başlangıç açısına ayarla (30°)
        hoodMotor.getEncoder().setPosition(ShooterConstants.kHoodHomeAngle);
        // ▶️ KRİTİK: Başlangıç açısını PID hedefi olarak hemen motorun içine yaz!
        // ▶️ CRITICAL: Immediately write the home angle as a PID target to the motor's
        // controller!
        // Bu motorun açılışta 0'a gitmeye çalışıp mekanik sınıra (30°) çarpmasını
        // önler.
        hoodMotor.getClosedLoopController().setSetpoint(ShooterConstants.kHoodHomeAngle, ControlType.kPosition);

        System.out.println("[Shooter] Hood: Onboard PID, Home=" + ShooterConstants.kHoodHomeAngle + "°");
    }

    private void configureFlywheelMotor() {
        // Velocity PID (RPS biriminde)
        // IMPORTANT: Verify kS, kV, kA (Feedforward) and kP are set in Constants!
        // These are critical for accurate Velocity Control on Kraken/TalonFX.
        flywheelConfig.Slot0.kP = flywheelKP.get();
        flywheelConfig.Slot0.kI = flywheelKI.get();
        flywheelConfig.Slot0.kD = flywheelKD.get();
        flywheelConfig.Slot0.kV = flywheelKV.get();

        // Motor ayarları
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 80;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        flywheelMotor.getConfigurator().apply(flywheelConfig);

        // --- Follower Configuration ---
        flywheelFollowerMotor.getConfigurator().apply(flywheelConfig); // Inherit current limits and neutral mode
        flywheelFollowerMotor.setControl(new Follower(flywheelMotor.getDeviceID(),
                followerInvert.get() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));

        System.out.println("[Shooter] Flywheel: Phoenix6 Velocity Control (Master-Follower Dual Kraken X60)");

        // Initialize Dashboard Toggle
        // Default to Clockwise_Positive (which corresponds to false/normal usually, but
        // let's check config)
        // Original config: flywheelConfig.MotorOutput.Inverted =
        // InvertedValue.Clockwise_Positive;
        // Let's assume this is the "Normal" (false) state for the toggle.
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Shooter/InvertFlywheel", false);

        // Initialize Auto-Aim Toggle (Boolean)
        // This allows using a Toggle Button or Checkbox on Dashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Shooter/EnableAutoAim", false);

        // Initialize Auto-Aim Inversion Toggle (Boolean)
        // Adds 180 degrees to the target angle (Failsafe)
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Tuning/Shooter/InvertAiming", false);
    }

    private boolean lastFlywheelInvertState = false;

    private void checkFlywheelInversion() {
        boolean invert = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Tuning/Shooter/InvertFlywheel",
                false);
        if (invert != lastFlywheelInvertState) {
            lastFlywheelInvertState = invert;

            // Toggle between Clockwise_Positive and CounterClockwise_Positive
            // Assuming Clockwise_Positive is the default "non-inverted" state based on
            // original code.
            // If invert is true -> CounterClockwise_Positive
            // If invert is false -> Clockwise_Positive

            InvertedValue newInvertValue = invert ? InvertedValue.CounterClockwise_Positive
                    : InvertedValue.Clockwise_Positive;

            flywheelConfig.MotorOutput.Inverted = newInvertValue;
            flywheelMotor.getConfigurator().apply(flywheelConfig);

            System.out.println("[Shooter] Flywheel Inversion Updated: " + newInvertValue);
        }
    }

    /**
     * Tuning modunda çalışırken PID değerlerini kontrol eder ve değişirse
     * günceller.
     */
    private void checkAndApplyTunables() {
        // Check Inversion
        checkFlywheelInversion();

        // --- TURRET PID UPDATE ---
        // --- TURRET PID UPDATE ---
        if (turretKP.hasChanged() || turretKI.hasChanged() || turretKD.hasChanged()) {
            turretConfig.closedLoop
                    .p(turretKP.get())
                    .i(turretKI.get())
                    .d(turretKD.get());
            turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Shooter] Turret PID Updated: P=" + turretKP.get() + ", I=" + turretKI.get() + ", D="
                    + turretKD.get());
        }

        // --- TURRET MAX OUTPUT (HIZ) UPDATE ---
        if (turretMaxOutput.hasChanged()) {
            turretConfig.closedLoop
                    .outputRange(-turretMaxOutput.get(), turretMaxOutput.get());
            turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Shooter] Turret Max Output Updated: " + turretMaxOutput.get());
        }

        // --- TURRET SOFT LIMITS UPDATE ---
        if (turretMinAngle.hasChanged() || turretMaxAngle.hasChanged()) {
            turretConfig.softLimit
                    .forwardSoftLimit((float) turretMaxAngle.get())
                    .reverseSoftLimit((float) turretMinAngle.get());
            turretMotor.configure(turretConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Shooter] Turret Soft Limits Updated: "
                    + turretMinAngle.get() + "° to " + turretMaxAngle.get() + "°");
        }

        // --- HOOD SOFT LIMIT UPDATE ---
        if (hoodMaxAngle.hasChanged()) {
            hoodConfig.softLimit.forwardSoftLimit((float) hoodMaxAngle.get());
            hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Shooter] Hood Forward Soft Limit Updated: " + hoodMaxAngle.get() + "°");
        }

        // --- HOOD PID UPDATE ---
        if (hoodKP.hasChanged() || hoodKI.hasChanged() || hoodKD.hasChanged()) {
            hoodConfig.closedLoop
                    .p(hoodKP.get())
                    .i(hoodKI.get())
                    .d(hoodKD.get());
            hoodMotor.configure(hoodConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println(
                    "[Shooter] Hood PID Updated: P=" + hoodKP.get() + ", I=" + hoodKI.get() + ", D=" + hoodKD.get());
        }

        // --- FLYWHEEL PID UPDATE ---
        if (flywheelKP.hasChanged() || flywheelKI.hasChanged() || flywheelKD.hasChanged() || flywheelKV.hasChanged()) {
            flywheelConfig.Slot0.kP = flywheelKP.get();
            flywheelConfig.Slot0.kI = flywheelKI.get();
            flywheelConfig.Slot0.kD = flywheelKD.get();
            flywheelConfig.Slot0.kV = flywheelKV.get();
            flywheelMotor.getConfigurator().apply(flywheelConfig);
            System.out.println("[Shooter] Flywheel PID Updated");
        }

        // --- FOLLOWER INVERSION UPDATE ---
        if (followerInvert.hasChanged()) {
            flywheelFollowerMotor.setControl(new Follower(flywheelMotor.getDeviceID(),
                    followerInvert.get() ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
            System.out.println("[Shooter] Follower Inversion Updated");
        }

        // --- MECHANICS UPDATE (Gear Ratios) ---
        if (turretGearRatio.hasChanged()) {
            configureTurretMotor();
            System.out.println("[Shooter] Turret Gear Ratio Updated: " + turretGearRatio.get());
        }
        if (hoodGearRatio.hasChanged()) {
            configureHoodMotor();
            System.out.println("[Shooter] Hood Gear Ratio Updated: " + hoodGearRatio.get());
        }
    }

    // =====================================================================
    // PERIODIC
    @Override
    public void periodic() {
        // Sync Dashboard Toggle for Auto-Aim
        boolean dashEnabled = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean(
                "Tuning/Shooter/EnableAutoAim",
                false);
        if (dashEnabled != isAutoAimActive) {
            if (dashEnabled) {
                enableAutoAim();
            } else {
                disableAutoAim();
            }
        }

        // IMPORTANT: Live Tuning ONLY in Test Mode
        // In Teleop/Autonomous, hardcoded constants are used
        if (edu.wpi.first.wpilibj.DriverStation.isTest()) {
            checkAndApplyTunables();
            checkFlywheelInversion();

        }
        // Live update calibration maps from Dashboard
        updateCalibrationMapsFromDashboard();

        // Continuous Auto-Aim Tracking

        // Read Offsets from Dashboard (Allow Manual Entry), Only update if Changed
        double dashTurret = tuningTable.getEntry("Offsets/Turret").getDouble(autoAimOffsetDeg);
        if (Math.abs(dashTurret - autoAimOffsetDeg) > 0.01)
            autoAimOffsetDeg = dashTurret;

        double dashHood = tuningTable.getEntry("Offsets/Hood").getDouble(hoodOffsetDeg);
        if (Math.abs(dashHood - hoodOffsetDeg) > 0.01)
            hoodOffsetDeg = dashHood;

        double dashFlywheel = tuningTable.getEntry("Offsets/Flywheel").getDouble(flywheelOffsetRPM);
        if (Math.abs(dashFlywheel - flywheelOffsetRPM) > 1.0)
            flywheelOffsetRPM = dashFlywheel;

        double dashHubX = tuningTable.getEntry("Offsets/HubX").getDouble(hubOffsetX);
        if (Math.abs(dashHubX - hubOffsetX) > 0.01)
            hubOffsetX = dashHubX;

        double dashHubY = tuningTable.getEntry("Offsets/HubY").getDouble(hubOffsetY);
        if (Math.abs(dashHubY - hubOffsetY) > 0.01)
            hubOffsetY = dashHubY;

        // Sync back
        tuningTable.getEntry("Offsets/Turret").setDouble(autoAimOffsetDeg);
        tuningTable.getEntry("Offsets/Hood").setDouble(hoodOffsetDeg);
        tuningTable.getEntry("Offsets/Flywheel").setDouble(flywheelOffsetRPM);
        tuningTable.getEntry("Offsets/HubX").setDouble(hubOffsetX);
        tuningTable.getEntry("Offsets/HubY").setDouble(hubOffsetY);

        // EMA velocity smoothing for moving shoot
        ChassisSpeeds rawSpeeds = fieldSpeedsSupplier.get();
        if (rawSpeeds != null) {
            double rawVx = rawSpeeds.vxMetersPerSecond;
            double rawVy = rawSpeeds.vyMetersPerSecond;
            if (Math.hypot(rawVx, rawVy) < 0.1) {
                // Robot durdu — residual hızı hemen sıfırla
                smoothedVx = 0.0;
                smoothedVy = 0.0;
            } else {
                double alpha = velocitySmoothAlpha.get();
                smoothedVx = alpha * rawVx + (1.0 - alpha) * smoothedVx;
                smoothedVy = alpha * rawVy + (1.0 - alpha) * smoothedVy;
            }
        }

        if (isAutoAimActive) {
            trackTarget();
        }

        // AdvantageKit Logging (always active for monitoring)
        logTelemetry();

        // Publish Status to Tuning Table (for Drivers)
        tuningTable.getEntry("Shooter/Ready").setBoolean(isReadyToShoot());
        tuningTable.getEntry("Shooter/AimLocked").setBoolean(isTurretAtTarget());
        tuningTable.getEntry("Shooter/FlywheelReady").setBoolean(isFlywheelAtTarget());
        tuningTable.getEntry("Shooter/HoodReady").setBoolean(isHoodAtTarget());
    }

    /**
     * Updates aiming based on robot pose and speed.
     * Calculates distance and angle to the Hub, applying moving shoot virtual
     * targets if enabled.
     * 
     * @param robotPose   Current robot pose from DriveSubsystem
     * @param fieldSpeeds Current robot field-relative speeds
     */
    public void updateAiming(Pose2d robotPose, edu.wpi.first.math.kinematics.ChassisSpeeds fieldSpeeds) {
        updateAiming(robotPose, fieldSpeeds, true);
    }

    /**
     * @param applyFlywheel true ise flywheel'i de çalıştır (ShootCommand),
     *                      false ise sadece taret + hood izle (AutoAim tracking).
     *                      true = also spin flywheel (ShootCommand use),
     *                      false = only turret + hood tracking (AutoAim use).
     */
    public void updateAiming(Pose2d robotPose, edu.wpi.first.math.kinematics.ChassisSpeeds fieldSpeeds,
            boolean applyFlywheel) {
        // --- Taret saha pozisyonunu hesapla ---
        // --- Calculate Turret Field Position ---
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(getTurretCenterOfRotation().rotateBy(robotPose.getRotation()));

        // 1. İttifaka göre hedef Hub'ı belirle (FieldConstants'tan dinamik)
        // 1. Determine Target Hub based on Alliance (Dynamic from FieldConstants)
        var alliance = DriverStation.getAlliance();
        Translation2d hubLocation = frc.robot.constants.FieldConstants.getHubCenter(alliance);

        // Hub Pozisyon Offset'ini uygula
        // Apply Hub Position Offset
        hubLocation = hubLocation.plus(new Translation2d(hubOffsetX, hubOffsetY));

        // Statik mesafe (hareket telafisi öncesi referans)
        // Static distance (pre-compensation reference)
        double staticDistance = turretFieldPosition.getDistance(hubLocation);

        // =====================================================================
        // HAREKET HALİNDE ATIŞ FİZİK PIPELINE'I
        // MOVING SHOOT PHYSICS PIPELINE
        // =====================================================================
        double distanceForLookup = staticDistance; // Varsayılan: statik mesafe / Default: static distance
        double turretLeadAngleDeg = 0;
        Translation2d aimTarget = hubLocation; // Nişan alınacak nokta / Point to aim at

        // MovingShoot sonuçları (log için sakla)
        // MovingShoot results (store for logging)
        double msRadialVel = 0;
        double msTangentialVel = 0;
        double msToF = 0;
        double msEffectiveDist = 0;

        if (enableMovingShoot.get()) {
            // Smoothed velocity kullan — ham hız yerine EMA filtreli hız
            // Use smoothed velocity — EMA-filtered instead of raw
            ChassisSpeeds smoothed = new ChassisSpeeds(smoothedVx, smoothedVy, 0);
            // Tam fizik hesabını çağır
            // Call the full physics calculation
            frc.robot.util.MovingShootUtil.MovingShootResult msResult = frc.robot.util.MovingShootUtil.calculate(
                    hubLocation,
                    turretFieldPosition,
                    smoothed,
                    averageShotVelocityMps.get(),
                    mechanicalLatencyMs.get() / 1000.0, // ms → saniye / ms → seconds
                    turretResponseMs.get() / 1000.0, // ms → saniye / ms → seconds
                    enableLatencyComp.get(),
                    enableTurretLead.get(),
                    movingShootVelGain.get() // Hız kazanç çarpanı / Velocity gain
            );

            // Sonuçları ayıkla / Extract results
            aimTarget = msResult.virtualTarget();
            distanceForLookup = msResult.effectiveDistance();
            turretLeadAngleDeg = msResult.turretLeadAngleDeg();
            msRadialVel = msResult.radialVelocity();
            msTangentialVel = msResult.tangentialVelocity();
            msToF = msResult.timeOfFlight();
            msEffectiveDist = msResult.effectiveDistance();
        }

        // 2. Taret → sanal hedef geometrik mesafesi (sadece log için)
        // 2. Turret → virtual target geometric distance (for logging only)
        double geometricDistance = turretFieldPosition.getDistance(aimTarget);

        // 3. Taret Açısını Hesapla
        // 3. Calculate Turret Angle
        // Taret orijininden hedefe açı
        // Angle to target from turret origin
        double angleToTargetRad = Math.atan2(
                aimTarget.getY() - turretFieldPosition.getY(),
                aimTarget.getX() - turretFieldPosition.getX());

        // Robotun başlık açısı
        // Robot's heading
        double robotHeadingRad = robotPose.getRotation().getRadians();

        // Robota göre taret açısı (Hedef - RobotBaşlık)
        // Turret Angle relative to Robot (Target - RobotHeading)
        double targetTurretRad = angleToTargetRad - robotHeadingRad;

        // -PI ile PI arasında normalleştir
        // Normalize to -PI to PI
        targetTurretRad = MathUtil.angleModulus(targetTurretRad);

        // Dereceye çevir, offset ve öncüleme ekle (kullanıcı isteğine göre TERS)
        // Convert to Degrees, add offset and lead (INVERTED per user request)
        turretTargetDeg = -Math.toDegrees(targetTurretRad) + autoAimOffsetDeg - turretLeadAngleDeg;

        // Taret yumuşak limitlerine kırp
        // Clamp Turret to Soft Limits
        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretTargetDeg = MathUtil.clamp(turretTargetDeg,
                    turretMinAngle.get(),
                    turretMaxAngle.get());
        }

        // 4. Atıcı Durumunu Hesapla (RPM & Hood) — EFEKTİF mesafeden
        // 4. Calculate Shooter State (RPM & Hood) — from EFFECTIVE distance
        // Efektif mesafe, radyal hız telafisini içerir
        // Effective distance includes radial velocity compensation
        ShooterState state = calculateShooterState(distanceForLookup);
        flywheelTargetRPM = state.rpm;
        hoodTargetDeg = state.hoodAngleDeg;

        // 5. Hedef değerleri uygula
        // 5. Apply Setpoints
        setTurretAngle(turretTargetDeg);
        if (applyFlywheel) {
            // Flywheel her zaman güncellenir
            setFlywheelRPM(state.rpm + flywheelOffsetRPM);
        }
        // Hood: sadece mesafe önemli ölçüde değiştiğinde güncelle (hysteresis)
        // Hood: only update when distance changes significantly (hysteresis)
        if (Math.abs(distanceForLookup - lastHoodUpdateDistanceM) > HOOD_UPDATE_THRESHOLD_M) {
            setHoodAngle(state.hoodAngleDeg + hoodOffsetDeg);
            lastHoodUpdateDistanceM = distanceForLookup;
        }

        // =====================================================================
        // TELEMETRİ — Tüm fizik ara değerlerini logla
        // TELEMETRY — Log all physics intermediate values
        // =====================================================================
        Logger.recordOutput("Tuning/Shooter/Aiming/StaticDistance", staticDistance);
        Logger.recordOutput("Tuning/Shooter/Aiming/GeometricDistance", geometricDistance);
        Logger.recordOutput("Tuning/Shooter/Aiming/EffectiveDistance", distanceForLookup);
        Logger.recordOutput("Tuning/Shooter/Aiming/TargetTurretAngle", turretTargetDeg);
        Logger.recordOutput("Tuning/Shooter/Aiming/TurretError", turretTargetDeg - getTurretAngle());
        Logger.recordOutput("Tuning/Shooter/Aiming/AimTargetX", aimTarget.getX());
        Logger.recordOutput("Tuning/Shooter/Aiming/AimTargetY", aimTarget.getY());
        Logger.recordOutput("Tuning/Shooter/Aiming/HubX", hubLocation.getX());
        Logger.recordOutput("Tuning/Shooter/Aiming/HubY", hubLocation.getY());
        Logger.recordOutput("Tuning/Shooter/Aiming/OffsetDeg", autoAimOffsetDeg);
        Logger.recordOutput("Tuning/Shooter/Aiming/HubOffsetX", hubOffsetX);
        Logger.recordOutput("Tuning/Shooter/Aiming/HubOffsetY", hubOffsetY);

        // Moving Shoot ara değerleri / Moving Shoot intermediate values
        Logger.recordOutput("Tuning/Shooter/MovingShoot/Enabled", enableMovingShoot.get());
        Logger.recordOutput("Tuning/Shooter/MovingShoot/RadialVelocity", msRadialVel);
        Logger.recordOutput("Tuning/Shooter/MovingShoot/TangentialVelocity", msTangentialVel);
        Logger.recordOutput("Tuning/Shooter/MovingShoot/TimeOfFlight", msToF);
        Logger.recordOutput("Tuning/Shooter/MovingShoot/EffectiveDistance", msEffectiveDist);
        Logger.recordOutput("Tuning/Shooter/MovingShoot/TurretLeadDeg", turretLeadAngleDeg);
        Logger.recordOutput("Tuning/Shooter/MovingShoot/LatencyCompEnabled", enableLatencyComp.get());
        Logger.recordOutput("Tuning/Shooter/MovingShoot/TurretLeadEnabled", enableTurretLead.get());
    }

    /**
     * Overload to support old static aiming calls.
     */
    public void updateAiming(Pose2d robotPose) {
        updateAiming(robotPose, new edu.wpi.first.math.kinematics.ChassisSpeeds());
    }

    /**
     * Updates aiming but uses a MANUAL RPM value instead of distance-based RPM.
     * Turret and Hood are still automated.
     */
    public void updateAimingManualRPM(double manualRPM) {
        // 1. Get Robot Pose
        Pose2d robotPose = robotPoseSupplier.get();

        // --- Calculate Turret Field Position ---
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(getTurretCenterOfRotation().rotateBy(robotPose.getRotation()));

        // 2. Determine Target Hub based on Alliance
        var alliance = DriverStation.getAlliance();
        Translation2d hubLocation = frc.robot.constants.FieldConstants.getHubCenter(alliance);

        // Apply Hub Position Offset
        hubLocation = hubLocation.plus(new Translation2d(hubOffsetX, hubOffsetY));

        // 3. Calculate Distance from Turret
        double distance = turretFieldPosition.getDistance(hubLocation);

        // 4. Calculate Turret Angle
        double angleToTargetRad = Math.atan2(
                hubLocation.getY() - turretFieldPosition.getY(),
                hubLocation.getX() - turretFieldPosition.getX());

        double robotHeadingRad = robotPose.getRotation().getRadians();
        double targetTurretRad = angleToTargetRad - robotHeadingRad;
        targetTurretRad = MathUtil.angleModulus(targetTurretRad);

        turretTargetDeg = Math.toDegrees(targetTurretRad) + autoAimOffsetDeg;

        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretTargetDeg = MathUtil.clamp(turretTargetDeg,
                    turretMinAngle.get(),
                    turretMaxAngle.get());
        }

        // 5. Calculate Hood Angle based on distance
        ShooterState state = calculateShooterState(distance);
        hoodTargetDeg = state.hoodAngleDeg;

        // 6. Use MANUAL RPM
        flywheelTargetRPM = manualRPM;

        // 7. Apply Setpoints
        setTurretAngle(turretTargetDeg);
        setHoodAngle(hoodTargetDeg);
        setFlywheelRPM(flywheelTargetRPM);

        // Log Data
        Logger.recordOutput("Tuning/Shooter/Aiming/Distance", distance);
        Logger.recordOutput("Tuning/Shooter/Aiming/TargetTurretAngle", turretTargetDeg);
        Logger.recordOutput("Tuning/Shooter/Aiming/ManualRPM", manualRPM);
    }

    /**
     * Updates aiming for ALLIANCE PASS (Shooting to Feeding Station area).
     * Calculates distance and angle to the Pass Target.
     * 
     * @param robotPose Current robot pose
     */
    public void updateAimingForPass(Pose2d robotPose) {
        updateAimingForPass(robotPose, new ChassisSpeeds(), true);
    }

    public void updateAimingForPass(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
        updateAimingForPass(robotPose, fieldSpeeds, true);
    }

    /**
     * Alliance Pass hedeflemesini günceller — hareket telafisi destekli.
     * Updates Alliance Pass aiming — with moving shoot compensation.
     *
     * @param robotPose     Robotun güncel pozisyonu / Current robot pose
     * @param fieldSpeeds   Robotun saha-referanslı hız vektörü / Robot
     *                      field-relative velocity
     * @param applyFlywheel true = flywheel'i de çalıştır (ShootCommand); false =
     *                      sadece taret+hood (AutoAim)
     *                      true = also spin flywheel (ShootCommand); false =
     *                      turret+hood only (AutoAim)
     */
    public void updateAimingForPass(Pose2d robotPose, ChassisSpeeds fieldSpeeds, boolean applyFlywheel) {
        // --- Taret saha pozisyonunu hesapla ---
        // --- Calculate Turret Field Position ---
        Translation2d turretFieldPosition = robotPose.getTranslation()
                .plus(getTurretCenterOfRotation().rotateBy(robotPose.getRotation()));

        // 1. Pass Hedefini Belirle (En Yakın Sol/Sağ)
        // 1. Determine Pass Target (Closest Left/Right)
        var alliance = DriverStation.getAlliance();
        Translation2d targetLocation = FieldConstants.getPassTarget(alliance, robotPose);

        // Hareket telafisi uygula / Apply moving shoot compensation
        double distanceForLookup;
        Translation2d aimTarget = targetLocation;

        if (enableMovingShoot.get() && fieldSpeeds != null) {
            frc.robot.util.MovingShootUtil.MovingShootResult msResult = frc.robot.util.MovingShootUtil.calculate(
                    targetLocation,
                    turretFieldPosition,
                    fieldSpeeds,
                    averageShotVelocityMps.get(),
                    mechanicalLatencyMs.get() / 1000.0,
                    turretResponseMs.get() / 1000.0,
                    enableLatencyComp.get(),
                    enableTurretLead.get(),
                    movingShootVelGain.get());
            aimTarget = msResult.virtualTarget();
            distanceForLookup = msResult.effectiveDistance();
        } else {
            distanceForLookup = turretFieldPosition.getDistance(targetLocation);
        }

        // 2. Taret Açısını Hesapla / Calculate Turret Angle
        double angleToTargetRad = Math.atan2(
                aimTarget.getY() - turretFieldPosition.getY(),
                aimTarget.getX() - turretFieldPosition.getX());

        double robotHeadingRad = robotPose.getRotation().getRadians();
        double targetTurretRad = angleToTargetRad - robotHeadingRad;
        targetTurretRad = MathUtil.angleModulus(targetTurretRad);

        turretTargetDeg = Math.toDegrees(targetTurretRad);

        // Taret limitlerine kırp / Clamp Turret
        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretTargetDeg = MathUtil.clamp(turretTargetDeg,
                    turretMinAngle.get(),
                    turretMaxAngle.get());
        }

        // 3. Atıcı Durumunu Hesapla (Alliance Pass mantığı) — efektif mesafe kullan
        // 3. Calculate Shooter State (Alliance Pass Logic) — use effective distance
        ShooterState state = calculateShooterStateForAlliancePass(distanceForLookup);
        hoodTargetDeg = state.hoodAngleDeg;

        // 4. Hedef değerleri uygula / Apply Setpoints
        setTurretAngle(turretTargetDeg);
        if (applyFlywheel) {
            flywheelTargetRPM = state.rpm;
            applyShooterState(state);
        } else {
            // Sadece hood izle — flywheel şu an çalışmıyor
            // Only track hood — flywheel not running yet
            setHoodAngle(hoodTargetDeg + hoodOffsetDeg);
        }

        // Telemetri / Log Aiming Data
        Logger.recordOutput("Tuning/Shooter/PassAiming/Distance", distanceForLookup);
        Logger.recordOutput("Tuning/Shooter/PassAiming/TargetTurretAngle", turretTargetDeg);
    }

    /**
     * Checks if the shooter is ready to fire based on PID errors.
     * TRUSTS POSE ESTIMATION AND PID LOOPS.
     * 
     * @return true if RPM, Hood, and Turret are within tolerance.
     */
    public boolean isReadyToShoot() {
        // Check 1: RPM Tolerance
        boolean rpmReady = Math.abs(getFlywheelRPM() - flywheelTargetRPM) < ShooterConstants.SHOOTER_RPM_TOLERANCE
                .get();

        // Check 2: Hood Tolerance
        boolean hoodReady = Math.abs(getHoodAngle() - hoodTargetDeg) < ShooterConstants.HOOD_ANGLE_TOLERANCE.get();

        // Check 3: Turret Tolerance (PID Error)
        // We compare current angle to target angle
        boolean turretReady = Math.abs(getTurretAngle() - turretTargetDeg) < ShooterConstants.TURRET_AIM_TOLERANCE
                .get();

        // Return true ONLY if ALL checks pass
        return rpmReady && hoodReady && turretReady;
    }

    // =====================================================================
    // TURRET CONTROL (Derece) - setSetpoint API
    // =====================================================================
    /**
     * Turret hedef açısını ayarlar (derece).
     * Turret center offset otomatik olarak uygulanır.
     * REVLib onboard PID kullanır.
     * 
     * @param angleDeg Hedef açı (mekanik merkeze göre, derece)
     */
    public void setTurretAngle(double angleDeg) {
        turretTargetDeg = MathUtil.clamp(angleDeg,
                turretMinAngle.get(),
                turretMaxAngle.get());

        // Center offset'i encoder pozisyonuna çevir:
        // Encoder 0° = mekanik merkez + offset
        double encoderTarget = turretTargetDeg + turretCenterOffset.get();

        // setSetpoint ile onboard PID'e hedef gönder
        turretMotor.getClosedLoopController().setSetpoint(encoderTarget, ControlType.kPosition);
    }

    /**
     * Turret'in mevcut açısını döndürür (mekanik merkeze göre, derece).
     * Center offset otomatik olarak çıkarılır.
     */
    public double getTurretAngle() {
        return turretMotor.getEncoder().getPosition() - turretCenterOffset.get();
    }

    /**
     * Turret hedefe ulaştı mı?
     */
    public boolean isTurretAtTarget() {
        return Math.abs(getTurretAngle() - turretTargetDeg) < ShooterConstants.TURRET_AIM_TOLERANCE.get();
    }

    // =====================================================================
    // HOOD CONTROL (Derece) - setSetpoint API
    // =====================================================================
    /**
     * Hood hedef açısını ayarlar (derece).
     * REVLib onboard PID kullanır.
     * 
     * @param angleDeg Hedef açı (min-max arası)
     */
    public void setHoodAngle(double angleDeg) {
        hoodTargetDeg = MathUtil.clamp(angleDeg,
                ShooterConstants.kHoodMinAngle,
                hoodMaxAngle.get());

        // setSetpoint ile onboard PID'e hedef gönder
        hoodMotor.getClosedLoopController().setSetpoint(hoodTargetDeg, ControlType.kPosition);
    }

    /**
     * Hood'un mevcut açısını döndürür (derece).
     */
    public double getHoodAngle() {
        return hoodMotor.getEncoder().getPosition();
    }

    /**
     * Hood hedefe ulaştı mı?
     */
    public boolean isHoodAtTarget() {
        return Math.abs(getHoodAngle() - hoodTargetDeg) < ShooterConstants.HOOD_ANGLE_TOLERANCE.get();
    }

    // =====================================================================
    // FLYWHEEL CONTROL (RPM)
    // =====================================================================
    /**
     * Flywheel hedef hızını ayarlar (RPM).
     * 
     * @param rpm Hedef hız
     */
    public void setFlywheelRPM(double rpm) {
        flywheelTargetRPM = Math.max(0, rpm);

        // RPM → RPS dönüşümü (Phoenix6 RPS kullanır)
        double rps = flywheelTargetRPM / 60.0;
        flywheelMotor.setControl(flywheelVelocity.withVelocity(rps));
    }

    /**
     * Flywheel'in mevcut hızını döndürür (RPM).
     */
    public double getFlywheelRPM() {
        return flywheelMotor.getVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * Flywheel hedefe ulaştı mı?
     */
    public boolean isFlywheelAtTarget() {
        return Math.abs(getFlywheelRPM() - flywheelTargetRPM) < flywheelTolerance.get();
    }

    /**
     * Flywheel'i durdurur.
     */
    public void stopFlywheel() {
        flywheelTargetRPM = 0;
        flywheelMotor.stopMotor();
    }

    // =====================================================================
    // AUTO-AIM SYSTEM
    // =====================================================================

    /**
     * Toggles the Auto-Aim state.
     */
    public void toggleAutoAim() {
        if (isAutoAimActive) {
            disableAutoAim();
        } else {
            enableAutoAim();
        }
    }

    /**
     * Auto-aim'i etkinleştirir.
     * Turret ve hood otomatik olarak hub'a yönelir.
     */
    public void enableAutoAim() {
        isAutoAimActive = true;
        // Sync with Dashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Tuning/Shooter/EnableAutoAim", true);
        tuningTable.getEntry("Tuning/Shooter/EnableAutoAim").setBoolean(true);
        System.out.println("[Shooter] Auto-aim ENABLED - Alliance: " + getAllianceString());
    }

    /**
     * Auto-aim'i devre dışı bırakır.
     */
    public void disableAutoAim() {
        isAutoAimActive = false;
        // Dashboard ile senkronize et / Sync with Dashboard
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Tuning/Shooter/EnableAutoAim", false);
        tuningTable.getEntry("Tuning/Shooter/EnableAutoAim").setBoolean(false);
        System.out.println("[Shooter] Auto-aim KAPALI / DISABLED — Taret son konumda kalıyor.");
        // NOT: Tureti 0'a almıyoruz — son hedefleme konumunda bırakıyoruz.
        // NOTE: We do NOT reset turret to 0 — leaving it at last aim position.
    }

    // =====================================================================
    // TUNABLES FOR TARGET CORRECTION (Offsets)
    // =====================================================================
    private final TunableNumber tunableTargetOverrideX = new TunableNumber("Shooter/Target", "OverrideX", 0);
    private final TunableNumber tunableTargetOverrideY = new TunableNumber("Shooter/Target", "OverrideY", 0);

    // Live Offsets (Reset on Code Restart)
    private double hoodOffsetDeg = 0.0;
    private double flywheelOffsetRPM = 0.0;
    // autoAimOffsetDeg already exists for Turret

    /**
     * Adjusts the Hood Offset (Angle).
     * 
     * @param deltaDeg Amount to change (e.g. +1.0 or -1.0)
     */
    public void adjustHoodOffset(double deltaDeg) {
        this.hoodOffsetDeg += deltaDeg;
    }

    /**
     * Adjusts the Flywheel Offset (RPM).
     * 
     * @param deltaRPM Amount to change (e.g. +100 or -100)
     */
    public void adjustFlywheelOffset(double deltaRPM) {
        this.flywheelOffsetRPM += deltaRPM;
    }

    /**
     * Increase Hood Offset by 2 degrees.
     */
    public Command increaseHoodOffsetCommand() {
        return Commands.runOnce(() -> adjustHoodOffset(2.0))
                .ignoringDisable(true)
                .withName("HoodOffset +2");
    }

    /**
     * Decrease Hood Offset by 2 degrees.
     */
    public Command decreaseHoodOffsetCommand() {
        return Commands.runOnce(() -> adjustHoodOffset(-2.0))
                .ignoringDisable(true)
                .withName("HoodOffset -2");
    }

    public double getHoodOffset() {
        return hoodOffsetDeg;
    }

    public double getFlywheelOffset() {
        return flywheelOffsetRPM;
    }

    private void trackTarget() {
        // Null guard: pose veya hız supplier henüz hazır değilse atla
        // Null guard: skip if pose or speed supplier not ready yet
        Pose2d pose = robotPoseSupplier.get();
        if (pose == null) {
            Logger.recordOutput("Tuning/Shooter/AutoAim/PoseNull", true);
            return;
        }
        Logger.recordOutput("Tuning/Shooter/AutoAim/PoseNull", false);

        ChassisSpeeds speeds = fieldSpeedsSupplier != null ? fieldSpeedsSupplier.get() : new ChassisSpeeds();
        if (speeds == null)
            speeds = new ChassisSpeeds();

        // AutoAim izleme modunda flywheel KAPATIK — sadece taret + hood hedefler
        // In AutoAim tracking mode flywheel OFF — only turret + hood track target
        // Flywheel sadece ShootCommand/AutoShootCommand aktifken çalışır.
        // Flywheel only runs when ShootCommand/AutoShootCommand is active.
        if (isInAllianceZone()) {
            updateAiming(pose, speeds, false); // applyFlywheel = false
        } else {
            updateAimingForPass(pose, speeds, false); // applyFlywheel = false
        }
    }

    /**
     * Auto-aim aktif mi?
     */
    public boolean isAutoAimActive() {
        return isAutoAimActive;
    }

    /**
     * Mevcut ittifakın hedef hub pozisyonunu döndürür.
     * Blue alliance → Blue Hub (sol taraf)
     * Red alliance → Red Hub (sağ taraf)
     */
    public Translation2d getTargetHub() {
        var alliance = DriverStation.getAlliance();
        Translation2d hubLocation = frc.robot.constants.FieldConstants.getHubCenter(alliance);

        // --- OVERRIDE LOGIC START ---
        double ovX = tunableTargetOverrideX.get();
        double ovY = tunableTargetOverrideY.get();

        if (Math.abs(ovX) > 0.01 || Math.abs(ovY) > 0.01) {
            hubLocation = new Translation2d(ovX, ovY);
            Logger.recordOutput("Shooter/Target/IsOverridden", true);
        } else {
            Logger.recordOutput("Shooter/Target/IsOverridden", false);
        }
        // --- OVERRIDE LOGIC END ---

        return hubLocation;
    }

    /**
     * Taretin robot merkezine göre fiziksel pozisyonunu (dönüş merkezi) döndürür.
     */
    public Translation2d getTurretCenterOfRotation() {
        return new Translation2d(turretPositionX.get(), turretPositionY.get());
    }

    /**
     * Mevcut ittifakı string olarak döndürür.
     */
    private String getAllianceString() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get().toString();
        }
        return "Unknown (defaulting to Blue)";
    }

    /**
     * Robot pozisyonundan hub'a olan mesafeyi hesaplar (metre).
     */
    public double getDistanceToHub() {
        return getDistanceToPoint(getTargetHub());
    }

    /**
     * Robotun Alliance Zone içinde olup olmadığını kontrol eder.
     */
    public boolean isInAllianceZone() {
        Pose2d robotPose = robotPoseSupplier.get();
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            // Red Alliance: X > MinX (12.49)
            return robotPose.getX() >= FieldConstants.kRedAllianceZoneMinX;
        } else {
            // Blue Alliance: X < MaxX (4.0)
            return robotPose.getX() <= FieldConstants.kBlueAllianceZoneMaxX;
        }
    }

    /**
     * Feeding için hedef noktayı döndürür.
     */
    public Translation2d getFeedingTarget() {
        var alliance = DriverStation.getAlliance();
        return FieldConstants.getFeedingTarget(alliance);
    }

    /**
     * Robot pozisyonundan belirli bir noktaya olan mesafeyi hesaplar.
     */
    public double getDistanceToPoint(Translation2d point) {
        return robotPoseSupplier.get().getTranslation().getDistance(point);
    }

    /**
     * Auto-aim offset değerini ayarlar (derece).
     * Pozitif: Sola kaydır, Negatif: Sağa kaydır (veya tam tersi, deneme ile
     * belirlenir)
     */
    public void setAutoAimOffset(double offsetDeg) {
        this.autoAimOffsetDeg = offsetDeg;
    }

    /**
     * Auto-aim offset değerini artırır/azaltır.
     */
    public void adjustAutoAimOffset(double deltaDeg) {
        this.autoAimOffsetDeg += deltaDeg;
    }

    /**
     * Mevcut auto-aim offset değerini döndürür.
     */
    public double getAutoAimOffset() {
        return autoAimOffsetDeg;
    }

    /**
     * Hub pozisyon offset'ini artırır/azaltır (metre).
     * POV tuşları ile çağrılır.
     * 
     * @param deltaX X offset değişimi (metre, saha ileri/geri)
     * @param deltaY Y offset değişimi (metre, saha sol/sağ)
     */
    public void adjustHubOffset(double deltaX, double deltaY) {
        this.hubOffsetX += deltaX;
        this.hubOffsetY += deltaY;
        System.out.println("[Shooter] Hub Offset: X=" + hubOffsetX + "m, Y=" + hubOffsetY + "m");
    }

    /**
     * Hub pozisyon offset'ini sıfırlar (0.0).
     */
    public void resetHubOffset() {
        this.hubOffsetX = 0.0;
        this.hubOffsetY = 0.0;
        System.out.println("[Shooter] Hub Offset RESET (0,0)");
    }

    /**
     * Hub X offset değerini döndürür (metre).
     */
    public double getHubOffsetX() {
        return hubOffsetX;
    }

    /**
     * Hub Y offset değerini döndürür (metre).
     */
    public double getHubOffsetY() {
        return hubOffsetY;
    }

    // =====================================================================
    // TELEMETRY (AdvantageKit)
    // =====================================================================
    private void logTelemetry() {
        // Core state - sadece önemli değerler
        Logger.recordOutput("Tuning/Shooter/ActualDeg", getTurretAngle());
        Logger.recordOutput("Tuning/Shooter/Hood/ActualDeg", getHoodAngle());
        Logger.recordOutput("Tuning/Shooter/Hood/ErrorDeg", hoodTargetDeg - getHoodAngle());
        Logger.recordOutput("Tuning/Shooter/Flywheel/ActualRPM", getFlywheelRPM());
        Logger.recordOutput("Tuning/Shooter/AutoAimActive", isAutoAimActive);
        Logger.recordOutput("Tuning/Shooter/AutoAim/OffsetDeg", autoAimOffsetDeg);

        if (Constants.tuningMode) {
            Logger.recordOutput("Tuning/Shooter/Turret/TargetDeg", turretTargetDeg);
            Logger.recordOutput("Tuning/Shooter/Hood/TargetDeg", hoodTargetDeg);
            Logger.recordOutput("Tuning/Shooter/Flywheel/TargetRPM", flywheelTargetRPM);

            Logger.recordOutput("Tuning/Shooter/Turret/MinAngle", turretMinAngle.get());
            Logger.recordOutput("Tuning/Shooter/Turret/MaxAngle", turretMaxAngle.get());
            Logger.recordOutput("Tuning/Shooter/TurretGearRatio", turretGearRatio.get());
            Logger.recordOutput("Tuning/Shooter/HoodGearRatio", hoodGearRatio.get());
        }
    }

    // =====================================================================
    // MODE SETTERS
    // =====================================================================
    public void setMode(ShooterMode mode) {
        this.currentMode = mode;
    }

    public ShooterMode getMode() {
        return currentMode;
    }

    // =====================================================================
    // TEST MODE METHODS
    // =====================================================================
    /**
     * Test modunda turret açısını ayarlar.
     */
    public void setTurretAngleTest(double angleDeg) {
        setTurretAngle(angleDeg);
    }

    /**
     * Test modunda hood açısını ayarlar.
     */
    public void setHoodAngleTest(double angleDeg) {
        setHoodAngle(angleDeg);
    }

    // =====================================================================
    // COMPATIBILITY METHODS
    // =====================================================================

    public double getFlywheelActualRPM() {
        return getFlywheelRPM();
    }

    public double getTurretActualAngle() {
        return getTurretAngle();
    }

    public double getHoodActualAngle() {
        return getHoodAngle();
    }

    public void stopShooter() {
        stopFlywheel();
        currentMode = ShooterMode.IDLE;
    }

    public void shoot() {
        currentMode = ShooterMode.SCORING;
    }

    public boolean isAimingAtTarget() {
        return isTurretAtTarget() && isHoodAtTarget() && isFlywheelAtTarget();
    }

    public void logSensorTelemetry() {
        logTelemetry();
    }

    // =====================================================================
    // SIMULATION
    // =====================================================================
    private edu.wpi.first.wpilibj.simulation.FlywheelSim flySim;
    private edu.wpi.first.wpilibj.simulation.SingleJointedArmSim turretSim;
    private edu.wpi.first.wpilibj.simulation.SingleJointedArmSim hoodSim;

    // Simülasyon görselleştirme için Mechanism2d (Opsiyonel ama yararlı)
    // Şu an sadece fiziksel değerleri güncelleyeceğiz.

    @Override
    public void simulationPeriodic() {
        // --- 1. Initialize Sims if null ---
        if (flySim == null) {
            setupSimulation();
        }

        // --- 2. Update Inputs (Voltage) ---
        // Flywheel
        double flywheelVoltage = flywheelMotor.getSimState().getMotorVoltage();
        flySim.setInput(flywheelVoltage);

        // Turret & Hood (SparkMax)
        // AppliedOutput -1..1 * BusVoltage assumed 12V
        // Getting actual bus voltage might be better but 12V is standard for Sim.
        double turretVoltage = turretMotor.getAppliedOutput() * 12.0;
        turretSim.setInput(turretVoltage);

        double hoodVoltage = hoodMotor.getAppliedOutput() * 12.0;
        hoodSim.setInput(hoodVoltage);

        // --- 3. Update Physics (0.02s dt) ---
        flySim.update(0.02);
        turretSim.update(0.02);
        hoodSim.update(0.02);

        // --- 4. Update Sim State (Sensors) ---

        // Flywheel: Phoenix6 SimState
        // Sim calculates RPM -> Convert to RPS
        double flyRPS = flySim.getAngularVelocityRPM() / 60.0;
        flywheelMotor.getSimState().setRotorVelocity(flyRPS);
        flywheelMotor.getSimState().setSupplyVoltage(12.0);

        // Follower must also have its sensor updated.
        // If it's Opposed, its sensor rotates in the opposite direction of the
        // mechanism.
        double followerRPS = followerInvert.get() ? -flyRPS : flyRPS;
        flywheelFollowerMotor.getSimState().setRotorVelocity(followerRPS);
        flywheelFollowerMotor.getSimState().setSupplyVoltage(12.0);

        // Turret: SparkMax Encoder
        // Sim calculates Radians -> Convert to Degrees
        double turretDeg = Math.toDegrees(turretSim.getAngleRads());
        turretMotor.getEncoder().setPosition(turretDeg);

        // Hood: SparkMax Encoder
        double hoodDeg = Math.toDegrees(hoodSim.getAngleRads());
        hoodMotor.getEncoder().setPosition(hoodDeg);
    }

    private void setupSimulation() {
        // Flywheel
        flySim = new edu.wpi.first.wpilibj.simulation.FlywheelSim(
                edu.wpi.first.math.system.plant.LinearSystemId.createFlywheelSystem(
                        edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(2), 0.001, 1.0),
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(2), 1.0);

        // Turret
        turretSim = new edu.wpi.first.wpilibj.simulation.SingleJointedArmSim(
                edu.wpi.first.math.system.plant.LinearSystemId.createSingleJointedArmSystem(
                        edu.wpi.first.math.system.plant.DCMotor.getNEO(1), 0.1, ShooterConstants.kTurretGearRatio),
                edu.wpi.first.math.system.plant.DCMotor.getNEO(1), ShooterConstants.kTurretGearRatio, 0.2,
                Math.toRadians(ShooterConstants.kTurretMinAngle), Math.toRadians(ShooterConstants.kTurretMaxAngle),
                false, 0.0);

        // Hood
        hoodSim = new edu.wpi.first.wpilibj.simulation.SingleJointedArmSim(
                edu.wpi.first.math.system.plant.LinearSystemId.createSingleJointedArmSystem(
                        edu.wpi.first.math.system.plant.DCMotor.getNEO(1), 0.05, ShooterConstants.kHoodGearRatio),
                edu.wpi.first.math.system.plant.DCMotor.getNEO(1), ShooterConstants.kHoodGearRatio, 0.1,
                Math.toRadians(ShooterConstants.kHoodMinAngle), Math.toRadians(ShooterConstants.kHoodMaxAngle),
                false, Math.toRadians(ShooterConstants.kHoodHomeAngle));
    }
    // --- AUTO SHOOT TEST COMMAND (Dashboard) ---
    // AutoShootCommand'ı constructor'dan çağırabilmek için lazy factory
    // kullanıyoruz.
    // FeederSubsystem dışarıdan enjekte edilemiyor, bu yüzden komutu
    // RobotContainer initAutoShootCommand() çağrısıyla kurar.

    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    /** Hood hysteresis'i sıfırla — ShootCommand başladığında çağır. */
    public void resetHoodUpdateDistance() {
        lastHoodUpdateDistanceM = -1.0;
    }

    public void disableHoodReverseSoftLimit() {
        // İPTAL EDİLDİ: CAN overun (35ms gecikme) yaratıyordu.
        // Konfigürasyonda kHoodMinAngle - 2.0 yapılarak limit fiziksel stop'un gerisine
        // çekildi,
        // böylece homing sırasında yazılımsal limite çarpmadan mekanik stop'a
        // yaslanabiliyor.
    }

    public void enableHoodReverseSoftLimit() {
        // İPTAL EDİLDİ: CAN overrun önlemi.
    }

    public void setTurretVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    public void setTurretPercent(double percent) {
        turretMotor.set(percent);
    }

    public boolean isTurretAtHomeLimit() {
        return turretMotor.getReverseLimitSwitch().isPressed();
    }

    public void resetHoodEncoder() {
        // Enkoderi en alt limit olan 30 dereceye sıfırla
        hoodMotor.getEncoder().setPosition(ShooterConstants.kHoodMinAngle);
        hoodTargetDeg = ShooterConstants.kHoodMinAngle;
        // ▶️ Sync Setpoint to prevent jump
        hoodMotor.getClosedLoopController().setSetpoint(hoodTargetDeg, ControlType.kPosition);
        System.out.println("[Shooter] Hood encoder sıfırlandı → " + ShooterConstants.kHoodMinAngle + "°");
    }

    public void resetTurretEncoder() {
        turretMotor.getEncoder().setPosition(0.0);
        turretTargetDeg = 0.0;
        System.out.println("[Shooter] Turret encoder sıfırlandı → 0° / Turret encoder reset → 0°");
    }

    public double getTurretEncoderPosition() {
        return turretMotor.getEncoder().getPosition();
    }

    public void initAutoShootCommand(frc.robot.subsystems.feeder.FeederSubsystem feeder,
            frc.robot.subsystems.drive.DriveSubsystem drive,
            frc.robot.subsystems.intake.IntakeSubsystem intake,
            java.util.function.Supplier<edu.wpi.first.math.geometry.Pose2d> poseSupplier) {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData(
                "Tuning/Shooter/Test/AutoShoot",
                new frc.robot.commands.shooter.AutoShootCommand(this, feeder, drive, intake, poseSupplier)
                        .withName("AutoShoot"));
    }

    public void stopAll() {
        turretMotor.set(0);
        hoodMotor.set(0);
        stopFlywheel();
    }

    /**
     * Dashboard'dan ShootResetCommand'ı başlatmak için kaydeder.
     * Registers ShootResetCommand for Dashboard triggering.
     * Constructor'dan sonra RobotContainer'da çağrılmalı.
     * Should be called from RobotContainer after construction.ssss
     */

}
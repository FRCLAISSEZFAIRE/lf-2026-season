package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotMap;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.Constants;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

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

    // Flywheel: Phoenix6 velocity control
    private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0);

    // =====================================================================
    // STATE VARIABLES
    // =====================================================================
    private final Supplier<Pose2d> robotPoseSupplier;

    private ShooterMode currentMode = ShooterMode.IDLE;
    private boolean isAutoAimActive = false;

    // Target values (HEPSİ DERECE veya RPM)
    private double turretTargetDeg = 0;
    private double hoodTargetDeg = ShooterConstants.kHoodMidAngle;
    private double flywheelTargetRPM = ShooterConstants.kIdleFlywheelRPM;
    private double autoAimOffsetDeg = 0.0;

    // =====================================================================
    // CALIBRATION MAPS
    // =====================================================================
    private final InterpolatingDoubleTreeMap hoodCalibrationMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap flywheelCalibrationMap = new InterpolatingDoubleTreeMap();

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

    // =====================================================================
    // CONSTRUCTOR
    // =====================================================================
    public ShooterSubsystem(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;

        // --- TURRET MOTOR (NEO + Relative Encoder + Onboard PID) ---
        turretMotor = new SparkMax(RobotMap.kTurretMotorID, MotorType.kBrushless);
        configureTurretMotor();

        // --- HOOD MOTOR (NEO + Relative Encoder + Onboard PID) ---
        hoodMotor = new SparkMax(RobotMap.kHoodMotorID, MotorType.kBrushless);
        configureHoodMotor();

        // --- FLYWHEEL MOTOR (Kraken X60) ---
        flywheelMotor = new TalonFX(RobotMap.kShooterMasterID);
        configureFlywheelMotor();

        System.out.println("[Shooter] REVLib 2026 setSetpoint API ile yapılandırıldı");

        // --- CALIBRATION INITIALIZATION ---
        initializeCalibration();
    }

    // =====================================================================
    // MOTOR CONFIGURATION
    // =====================================================================
    // =====================================================================
    // MOTOR CONFIGURATION OBJECTS (Stored for runtime updates)
    // =====================================================================
    private final SparkMaxConfig turretConfig = new SparkMaxConfig();
    private final SparkMaxConfig hoodConfig = new SparkMaxConfig();
    private final TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

    // =====================================================================
    // MOTOR CONFIGURATION
    // =====================================================================
    // =====================================================================
    // CALIBRATION INITIALIZATION
    // =====================================================================
    private void initializeCalibration() {
        // --- HOOD CALIBRATION (Distance (m) -> Angle (deg)) ---
        hoodCalibrationMap.put(0.0, 48.0); // Min mesafe (maksimum açı)
        hoodCalibrationMap.put(1.0, 48.0); // 1m -> 48 derece (maksimum sınır)
        hoodCalibrationMap.put(3.0, 35.0); // Ara mesafe tahmini
        hoodCalibrationMap.put(5.0, 25.0); // 5m -> 25 derece
        hoodCalibrationMap.put(8.0, 10.0); // Uzak mesafe (daha düşük açı)

        // --- FLYWHEEL CALIBRATION (Distance (m) -> RPM) ---
        flywheelCalibrationMap.put(0.0, 2000.0); // Min mesafe
        flywheelCalibrationMap.put(1.0, 2500.0); // 1m -> 2500 RPM
        flywheelCalibrationMap.put(3.0, 3750.0); // Ara mesafe tahmini
        flywheelCalibrationMap.put(5.0, 5000.0); // 5m -> 5000 RPM
        flywheelCalibrationMap.put(8.0, 6000.0); // Uzak mesafe
    }

    // =====================================================================
    // MOTOR CONFIGURATION
    // =====================================================================
    private void configureTurretMotor() {
        // Relative encoder yapılandırması (dahili encoder)
        // Motor:Turret oranı 40:1 → 1 motor tur = 9 derece turret
        double degreesPerMotorRot = 360.0 / ShooterConstants.kTurretGearRatio; // 360/40 = 9 deg/motor-rot
        turretConfig.encoder
                .positionConversionFactor(degreesPerMotorRot)
                .velocityConversionFactor(degreesPerMotorRot / 60.0);

        // REVLib onboard PID (setSetpoint ile kullanılacak)
        turretConfig.closedLoop
                .p(turretKP.get())
                .i(turretKI.get())
                .d(turretKD.get())
                .outputRange(-0.2, 0.2); // %50 max output (güvenlik)

        // Motor ayarları
        turretConfig.inverted(false);
        turretConfig.idleMode(IdleMode.kBrake);

        // Soft limits (derece olarak)
        turretConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit((float) ShooterConstants.kTurretMaxAngle)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit((float) ShooterConstants.kTurretMinAngle);

        turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder'ı sıfırla - TARET 0° POZİSYONUNDA BAŞLATILMALI!
        turretMotor.getEncoder().setPosition(0);

        System.out.println("[Shooter] Turret: Onboard PID, Relative Encoder, Soft Limits ±180°");
        System.out.println("[Shooter] ⚠️ TARET 0° POZİSYONUNDA BAŞLATILMALI!");
    }

    private void configureHoodMotor() {
        // Relative encoder dönüşümü (motor rotasyonu → derece)
        double degreesPerMotorRot = 360.0 / ShooterConstants.kHoodGearRatio;
        hoodConfig.encoder
                .positionConversionFactor(degreesPerMotorRot)
                .velocityConversionFactor(degreesPerMotorRot / 60.0);

        // REVLib onboard PID
        hoodConfig.closedLoop
                .p(hoodKP.get())
                .i(hoodKI.get())
                .d(hoodKD.get())
                .outputRange(-0.2, 0.2); // Güvenlik limiti

        // Motor ayarları
        hoodConfig.inverted(true);
        hoodConfig.idleMode(IdleMode.kBrake);

        hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder'ı başlangıç açısına ayarla
        hoodMotor.getEncoder().setPosition(ShooterConstants.kHoodHomeAngle);

        System.out.println("[Shooter] Hood: Onboard PID, Home=" + ShooterConstants.kHoodHomeAngle + "°");
    }

    private void configureFlywheelMotor() {
        // Velocity PID (RPS biriminde)
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

        System.out.println("[Shooter] Flywheel: Phoenix6 Velocity Control");
    }

    /**
     * Tuning modunda çalışırken PID değerlerini kontrol eder ve değişirse
     * günceller.
     */
    private void checkAndApplyTunables() {
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
    }

    // =====================================================================
    // PERIODIC
    // =====================================================================
    @Override
    public void periodic() {
        // Auto-Aim Logic
        updateAutoAim();

        // Tuning Mode Logic
        if (Constants.tuningMode) {
            checkAndApplyTunables();

            // Dashboard Toggle Check
            boolean dashboardAutoAim = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
                    .getBoolean("Shooter/EnableAutoAim", isAutoAimActive);
            if (dashboardAutoAim != isAutoAimActive) {
                if (dashboardAutoAim) {
                    enableAutoAim();
                } else {
                    disableAutoAim();
                }
            }
        }

        // AdvantageKit Logging
        logTelemetry();
    }

    // =====================================================================
    // TURRET CONTROL (Derece) - setSetpoint API
    // =====================================================================
    /**
     * Turret hedef açısını ayarlar (derece).
     * REVLib onboard PID kullanır.
     * 
     * @param angleDeg Hedef açı (-180 ile +180 arası)
     */
    public void setTurretAngle(double angleDeg) {
        turretTargetDeg = MathUtil.clamp(angleDeg,
                ShooterConstants.kTurretMinAngle,
                ShooterConstants.kTurretMaxAngle);

        // setSetpoint ile onboard PID'e hedef gönder
        turretMotor.getClosedLoopController().setSetpoint(turretTargetDeg, ControlType.kPosition);
    }

    /**
     * Turret'in mevcut açısını döndürür (derece).
     */
    public double getTurretAngle() {
        return turretMotor.getEncoder().getPosition();
    }

    /**
     * Turret hedefe ulaştı mı?
     */
    public boolean isTurretAtTarget() {
        return turretMotor.getClosedLoopController().isAtSetpoint();
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
                ShooterConstants.kHoodMaxAngle);

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
        return hoodMotor.getClosedLoopController().isAtSetpoint();
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
     * Auto-aim'i etkinleştirir.
     * Turret ve hood otomatik olarak hub'a yönelir.
     */
    public void enableAutoAim() {
        isAutoAimActive = true;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/EnableAutoAim", true);
        System.out.println("[Shooter] Auto-aim ENABLED - Alliance: " + getAllianceString());
    }

    /**
     * Auto-aim'i devre dışı bırakır.
     */
    public void disableAutoAim() {
        isAutoAimActive = false;
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Shooter/EnableAutoAim", false);
        System.out.println("[Shooter] Auto-aim DISABLED");
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
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.kRedSpeakerPosition;
        }
        return FieldConstants.kBlueSpeakerPosition;
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
     * Hub'ı hedeflemek için gereken turret açısını hesaplar (derece).
     */
    public double calculateTurretAngleToHub() {
        return calculateTurretAngleToPoint(getTargetHub());
    }

    /**
     * Mesafeye göre hood açısını hesaplar (derece).
     * Daha uzak = daha düşük açı (düz atış)
     * Daha yakın = daha yüksek açı (parabolik atış)
     */
    public double calculateHoodAngleFromDistance(double distanceMeters) {
        // Interpolasyon tablosundan al
        return hoodCalibrationMap.get(distanceMeters);
    }

    /**
     * Mesafeye göre flywheel RPM hesaplar.
     * Daha uzak = daha yüksek RPM
     */
    public double calculateFlywheelRPMFromDistance(double distanceMeters) {
        // Interpolasyon tablosundan al
        return flywheelCalibrationMap.get(distanceMeters);
    }

    /**
     * Auto-aim güncelleme metodu.
     * Aktifse turret, hood ve flywheel'i otomatik hedefler.
     * Her periodic döngüsünde veya command'dan çağrılabilir.
     */
    public void updateAutoAim() {
        if (!isAutoAimActive) {
            return;
        }

        // 1. Hedef Belirleme (Scoring vs Feeding)
        Translation2d targetPoint;
        double hoodAngle;
        double flywheelRPM;

        boolean inZone = isInAllianceZone();

        if (inZone) {
            // --- SCORING MODE (Alliance Zone İçinde) ---
            targetPoint = getTargetHub();
            double distance = getDistanceToPoint(targetPoint);

            hoodAngle = calculateHoodAngleFromDistance(distance);
            flywheelRPM = calculateFlywheelRPMFromDistance(distance);

            if (Constants.tuningMode) {
                Logger.recordOutput("Shooter/AutoAim/Mode", "SCORING");
                Logger.recordOutput("Shooter/AutoAim/Distance", distance);
            }
        } else {
            // --- FEEDING MODE (Alliance Zone Dışında) ---
            targetPoint = getFeedingTarget();
            // Feeding için sabit değerler (veya mesafeye göre ayarlanabilir, şimdilik
            // sabit)
            hoodAngle = ShooterConstants.kFeedingHoodAngle;
            flywheelRPM = ShooterConstants.kFeedingFlywheelRPM;

            if (Constants.tuningMode) {
                Logger.recordOutput("Shooter/AutoAim/Mode", "FEEDING");
                Logger.recordOutput("Shooter/AutoAim/Distance", getDistanceToPoint(targetPoint));
            }
        }

        // 2. Turret Açısı Hesaplama
        double turretAngle = calculateTurretAngleToPoint(targetPoint);

        // 3. Hedefleri Uygula
        double rawTargetAngle = turretAngle + autoAimOffsetDeg;

        // Wrap Optimization Logic:
        // [-200, 200] aralığında en verimli açıyı (mevcut açıya en yakın) seç.
        double optimizedTurretAngle = optimizeTurretAngle(getTurretAngle(), rawTargetAngle);

        setTurretAngle(optimizedTurretAngle);
        setHoodAngle(hoodAngle);
        setFlywheelRPM(flywheelRPM);

        // Debug log (AdvantageKit)
        if (Constants.tuningMode) {
            Logger.recordOutput("Shooter/AutoAim/RawTargetAngle", rawTargetAngle); // Debug için raw
            Logger.recordOutput("Shooter/AutoAim/OptimizedTargetAngle", optimizedTurretAngle);
            Logger.recordOutput("Shooter/AutoAim/CalculatedHoodAngle", hoodAngle);
            Logger.recordOutput("Shooter/AutoAim/CalculatedRPM", flywheelRPM);
            Logger.recordOutput("Shooter/AutoAim/TargetX", targetPoint.getX());
            Logger.recordOutput("Shooter/AutoAim/TargetY", targetPoint.getY());

            // CONSOLE LOG (For explicit Sim debugging)
            // Sadece her 50 döngüde bir yaz ki konsol dolmasın (yaklaşık 1 sn)
            if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() % 1.0 < 0.02) {
                Pose2d p = robotPoseSupplier.get();
                System.out.printf(
                        "[AutoAim] %s | Pose: (%.2f, %.2f) | Target: (%.2f, %.2f) | Turret: %.2f -> %.2f%n",
                        inZone ? "SCORING" : "FEEDING",
                        p.getX(), p.getY(),
                        targetPoint.getX(), targetPoint.getY(),
                        getTurretAngle(), optimizedTurretAngle);
            }
        }
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
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.kRedFeedingTarget;
        }
        return FieldConstants.kBlueFeedingTarget;
    }

    /**
     * Robot pozisyonundan belirli bir noktaya olan mesafeyi hesaplar.
     */
    public double getDistanceToPoint(Translation2d point) {
        return robotPoseSupplier.get().getTranslation().getDistance(point);
    }

    /**
     * Belirli bir noktaya dönmek için gereken turret açısını hesaplar.
     */
    public double calculateTurretAngleToPoint(Translation2d targetPoint) {
        Pose2d robotPose = robotPoseSupplier.get();

        // Robot'tan hedefe vektör
        double dx = targetPoint.getX() - robotPose.getX();
        double dy = targetPoint.getY() - robotPose.getY();

        // Hedefe olan açı (saha koordinatlarında)
        double fieldAngleRad = Math.atan2(dy, dx);

        // Robot yönelimi
        double robotAngleRad = robotPose.getRotation().getRadians();

        // Turret açısı = Hedef açı - Robot açısı
        double turretAngleRad = MathUtil.angleModulus(fieldAngleRad - robotAngleRad);

        return Math.toDegrees(turretAngleRad);
    }

    /**
     * Turret için en uygun hedef açıyı hesaplar.
     * Hedef açı [-180, 180] aralığında gelir, ancak turret [-200, 200] dönebilir.
     * Eğer mevcut konumdan diğer tarafa (`wrap`) dönmek daha kısaysa ve limitler
     * içindeyse, onu seçer.
     * 
     * @param currentDeg    Mevcut turret açısı
     * @param targetDegBase Hedef açı (genellikle -180 ile 180 arası normalize
     *                      edilmiş)
     * @return Optimize edilmiş hedef açı
     */
    private double optimizeTurretAngle(double currentDeg, double targetDegBase) {
        // Hedefi -180 ile 180 arasına normalize et (emin olmak için)
        double normalizedTarget = MathUtil.angleModulus(Math.toRadians(targetDegBase));
        normalizedTarget = Math.toDegrees(normalizedTarget);

        // Adaylar:
        // 1. Normal hedef (örn: 170)
        // 2. +360 fazı (örn: 170 + 360 = 530 -> Limit dışı)
        // 3. -360 fazı (örn: 170 - 360 = -190 -> Limit içi olabilir)

        double[] candidates = {
                normalizedTarget,
                normalizedTarget + 360.0,
                normalizedTarget - 360.0
        };

        double bestTarget = currentDeg; // Varsayılan (hareket yok)
        double minError = Double.MAX_VALUE;
        boolean foundValid = false;

        for (double cand : candidates) {
            // Limit kontrolü
            if (cand >= ShooterConstants.kTurretMinAngle && cand <= ShooterConstants.kTurretMaxAngle) {
                double error = Math.abs(cand - currentDeg);
                if (error < minError) {
                    minError = error;
                    bestTarget = cand;
                    foundValid = true;
                }
            }
        }

        // Eğer geçerli bir aday bulunamazsa (nadiren olur, aralık 400 derece total),
        // en yakın sınıra clamp'le veya base target'ı kullan.
        if (!foundValid) {
            // Fallback: Clamp directly
            return MathUtil.clamp(normalizedTarget, ShooterConstants.kTurretMinAngle, ShooterConstants.kTurretMaxAngle);
        }

        return bestTarget;
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
     * Eski metod ismi için alias (geriye uyumluluk)
     */
    public Translation2d getCurrentTarget() {
        return getTargetHub();
    }

    /**
     * Eski metod ismi için alias (geriye uyumluluk)
     */
    public double calculateTurretAngleToTarget() {
        return calculateTurretAngleToHub();
    }

    // =====================================================================
    // TELEMETRY (AdvantageKit)
    // =====================================================================
    private void logTelemetry() {
        // Core state - sadece önemli değerler
        Logger.recordOutput("Shooter/Turret/ActualDeg", getTurretAngle());
        Logger.recordOutput("Shooter/Hood/ActualDeg", getHoodAngle());
        Logger.recordOutput("Shooter/Flywheel/ActualRPM", getFlywheelRPM());
        Logger.recordOutput("Shooter/AutoAimActive", isAutoAimActive);
        Logger.recordOutput("Shooter/AutoAim/OffsetDeg", autoAimOffsetDeg);

        if (Constants.tuningMode) {
            Logger.recordOutput("Shooter/Turret/TargetDeg", turretTargetDeg);
            Logger.recordOutput("Shooter/Hood/TargetDeg", hoodTargetDeg);
            Logger.recordOutput("Shooter/Flywheel/TargetRPM", flywheelTargetRPM);
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
                        edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 0.001, 1.0),
                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60Foc(1), 1.0);

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

    public void stopAll() {
        turretMotor.set(0);
        hoodMotor.set(0);
        stopFlywheel();
    }
}
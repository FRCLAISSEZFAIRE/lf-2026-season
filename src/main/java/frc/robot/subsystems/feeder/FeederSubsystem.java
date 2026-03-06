package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.FeederConstants;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

/**
 * REVLib 2026 tabanlı Feeder alt sistemi.
 * setSetpoint velocity control kullanır.
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>NEO motor + REVLib Onboard Velocity PID</li>
 * <li>Intake → Shooter arası transfer</li>
 * </ul>
 */
public class FeederSubsystem extends SubsystemBase {

    // =====================================================================
    // MOTOR CONTROLLER
    // =====================================================================
    private final SparkMax feederMotor;

    // =====================================================================
    // STATE
    // =====================================================================
    private double targetRPM = 0;

    // =====================================================================
    // TUNABLE PID
    // =====================================================================
    private final TunableNumber kP = new TunableNumber("Feeder", "kP", FeederConstants.kP);
    private final TunableNumber kI = new TunableNumber("Feeder", "kI", FeederConstants.kI);
    private final TunableNumber kD = new TunableNumber("Feeder", "kD", FeederConstants.kD);
    private final TunableNumber kFF = new TunableNumber("Feeder", "kFF", FeederConstants.kFF);

    // =====================================================================
    // TUNABLE SPEEDS (Voltaj kontrol - Dashboard'dan ayarlanabilir)
    // =====================================================================
    private final TunableNumber tunableFeedVoltage = new TunableNumber("Feeder", "Feed Voltage",
            FeederConstants.kFeedVoltage);
    private final TunableNumber tunableSlowFeedVoltage = new TunableNumber("Feeder", "Slow Feed Voltage",
            FeederConstants.kSlowFeedVoltage);
    private final TunableNumber tunableReverseVoltage = new TunableNumber("Feeder", "Reverse Voltage",
            FeederConstants.kReverseVoltage);

    // =====================================================================
    // MANUAL OVERRIDE (Tuning Modunda Bağımsız Test)
    // =====================================================================
    private boolean manualOverrideEnabled = false;
    private double manualOverrideRPM = 0;

    // =====================================================================
    // CONSTRUCTOR
    // =====================================================================
    public FeederSubsystem() {
        // --- FEEDER MOTOR (NEO + Velocity Control) ---
        feederMotor = new SparkMax(FeederConstants.kFeederMotorID, MotorType.kBrushless);
        configureMotor();

        System.out.println("[Feeder] Voltaj kontrol ile yapılandırıldı (encoder bağımsız)");
    }

    // =====================================================================
    // MOTOR CONFIGURATION
    // =====================================================================
    private final SparkMaxConfig feederConfig = new SparkMaxConfig();

    private void configureMotor() {
        // Encoder dönüşüm faktörü (RPM olarak okumak için)
        // NEO encoder: 42 CPR, motor shaft'ta
        feederConfig.encoder
                .velocityConversionFactor(1.0); // Motor RPM direkt okunur

        // REVLib Onboard Velocity PID
        feederConfig.closedLoop
                .p(kP.get())
                .i(kI.get())
                .d(kD.get())
                .velocityFF(kFF.get())
                .outputRange(-1, 1);

        // Motor ayarları
        feederConfig.inverted(true);
        feederConfig.idleMode(IdleMode.kCoast);
        feederConfig.smartCurrentLimit(FeederConstants.kCurrentLimit);

        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Dashboard Toggle
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.setDefaultBoolean("Feeder/Invert", false);
    }

    /**
     * PID/FF değerleri değiştiyse motora uygular.
     * Test ve Teleop modlarında çalışır.
     */
    private void checkAndApplyTunables() {
        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kFF.hasChanged()) {
            feederConfig.closedLoop
                    .p(kP.get())
                    .i(kI.get())
                    .d(kD.get())
                    .velocityFF(kFF.get());
            feederMotor.configure(feederConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Feeder] PID/FF Updated: P=" + kP.get()
                    + ", I=" + kI.get() + ", D=" + kD.get() + ", FF=" + kFF.get());
        }
    }

    private boolean lastInvertState = false;

    private void checkInversion() {
        boolean invert = edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getBoolean("Feeder/Invert", false);
        if (invert != lastInvertState) {
            lastInvertState = invert;
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(invert);
            // Apply ONLY inverted setting, keeping other parameters?
            // REVLib 2026 checks suggest we should re-apply the whole config or use
            // specific setter if available.
            // But `configure` with `kNoResetSafeParameters` is safer for partial updates if
            // we had the full config object.
            // Better to re-run configureMotor() but allow it to take a parameter or just
            // read the field.
            // Let's modify configureMotor to read the field, or just apply the invert here
            // since we use PersistMode.

            // Re-applying basic config to ensure consistency
            config.closedLoop
                    .p(kP.get())
                    .i(kI.get())
                    .d(kD.get())
                    .velocityFF(kFF.get())
                    .outputRange(-1, 1);
            config.idleMode(IdleMode.kBrake);
            config.smartCurrentLimit(FeederConstants.kCurrentLimit);
            config.encoder.velocityConversionFactor(1.0);

            config.inverted(invert);

            feederMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Feeder] Motor Inversion Updated: " + invert);
        }
    }

    // =====================================================================
    // PERIODIC
    // =====================================================================
    @Override
    public void periodic() {
        // PID/FF değişikliklerini her modda kontrol et (maç içinde de ayarlanabilir)
        checkAndApplyTunables();

        if (edu.wpi.first.wpilibj.DriverStation.isTest()) {
            checkInversion();
        }
        logTelemetry();
    }

    // =====================================================================
    // VOLTAGE CONTROL (Encoder bağımsız)
    // =====================================================================

    /**
     * Feeder'a voltaj gönderir (encoder gereksiz).
     * 
     * @param volts Voltaj (-12 ile +12 arası)
     */
    public void setFeedVoltage(double volts) {
        targetRPM = volts; // Telemetri için voltajı kaydet
        feederMotor.setVoltage(volts);
    }

    /**
     * Feeder hızını ayarlar (RPM - eski velocity kontrol, encoder gerekli).
     * 
     * @param rpm Hedef hız (pozitif = ileri, negatif = geri)
     */
    public void setVelocity(double rpm) {
        targetRPM = rpm;
        feederMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * İleri besleme (tunable voltajda).
     */
    public void feed() {
        if (!manualOverrideEnabled) {
            setFeedVoltage(tunableFeedVoltage.get());
        }
    }

    /**
     * Yavaş besleme (tunable voltajda).
     */
    public void feedSlow() {
        if (!manualOverrideEnabled) {
            setFeedVoltage(tunableSlowFeedVoltage.get());
        }
    }

    /**
     * Geri çıkarma (tunable voltajda).
     */
    public void reverse() {
        if (!manualOverrideEnabled) {
            setFeedVoltage(tunableReverseVoltage.get());
        }
    }

    /**
     * Durdur.
     */
    public void stop() {
        if (!manualOverrideEnabled) {
            targetRPM = 0;
            feederMotor.stopMotor();
        }
    }

    // =====================================================================
    // MANUAL OVERRIDE (Tuning Mode için Bağımsız Test)
    // =====================================================================

    /**
     * Manuel override'ı etkinleştirir.
     * Shooter çalışmasına gerek kalmadan feeder test edilebilir.
     */
    public void enableManualOverride() {
        manualOverrideEnabled = true;
        System.out.println("[Feeder] Manual override ENABLED - Shooter bağımsız test modu");
    }

    /**
     * Manuel override'ı devre dışı bırakır.
     * Normal shooter-feeder koordinasyonu devreye girer.
     */
    public void disableManualOverride() {
        manualOverrideEnabled = false;
        manualOverrideRPM = 0;
        feederMotor.stopMotor();
        System.out.println("[Feeder] Manual override DISABLED - Normal mod");
    }

    /**
     * Manuel override aktif mi?
     */
    public boolean isManualOverrideEnabled() {
        return manualOverrideEnabled;
    }

    /**
     * Manuel override modunda feeder voltajını ayarlar.
     * 
     * @param volts Hedef voltaj (tuning için)
     */
    public void setManualOverrideVoltage(double volts) {
        if (manualOverrideEnabled) {
            manualOverrideRPM = volts;
            feederMotor.setVoltage(volts);
        }
    }

    /**
     * Manuel override modunda feeder RPM'ini ayarlar (velocity PID).
     * 
     * @param rpm Hedef hız (tuning için)
     */
    public void setManualOverrideRPM(double rpm) {
        if (manualOverrideEnabled) {
            manualOverrideRPM = rpm;
            feederMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        }
    }

    /**
     * Manuel override modunda ileri çalıştır.
     */
    public void manualFeed() {
        setManualOverrideVoltage(tunableFeedVoltage.get());
    }

    /**
     * Manuel override modunda durdur.
     */
    public void manualStop() {
        if (manualOverrideEnabled) {
            manualOverrideRPM = 0;
        }
        feederMotor.stopMotor();
    }

    // =====================================================================
    // STATUS
    // =====================================================================

    /**
     * Mevcut motor hızını döndürür (RPM).
     */
    public double getVelocityRPM() {
        return feederMotor.getEncoder().getVelocity();
    }

    /**
     * Feeder hedefe ulaştı mı?
     */
    public boolean isAtTarget() {
        return Math.abs(getVelocityRPM() - targetRPM) < FeederConstants.kRPMTolerance;
    }

    /**
     * Feeder çalışıyor mu?
     */
    public boolean isRunning() {
        return Math.abs(getVelocityRPM()) > 50;
    }

    // =====================================================================
    // TELEMETRY
    // =====================================================================
    private void logTelemetry() {
        Logger.recordOutput("Tuning/Feeder/TargetRPM", targetRPM);
        Logger.recordOutput("Tuning/Feeder/ActualRPM", getVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/ManualOverrideEnabled", manualOverrideEnabled);
        Logger.recordOutput("Tuning/Feeder/ManualOverrideRPM", manualOverrideRPM);
    }

    // =====================================================================
    // COMPATIBILITY METHODS
    // =====================================================================

    public double getTunableFeedVoltage() {
        return FeederConstants.kFeedRPM; // RPM olarak döner artık
    }

    public double getTunableReverseVoltage() {
        return FeederConstants.kReverseRPM;
    }

    /**
     * Manuel voltaj kontrolü (legacy uyumluluk için).
     * Tercih edilen: setVelocity()
     */
    public void setVoltage(double volts) {
        feederMotor.setVoltage(volts);
    }
}

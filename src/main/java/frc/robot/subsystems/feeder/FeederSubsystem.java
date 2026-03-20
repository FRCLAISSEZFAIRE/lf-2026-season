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
 * REVLib 2026 tabanlı Feeder alt sistemi — İkili Motor (Indexer + Kicker).
 * Her iki motor da SparkMax NEO v1.1, velocity PID ile kontrol edilir.
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Indexer: Topuğu shooter bölgesine indeksler</li>
 * <li>Kicker: Topuğu fırlatma bölgesine iter</li>
 * <li>REVLib Onboard Velocity PID (her motor ayrı)</li>
 * <li>Intake → Shooter arası transfer</li>
 * </ul>
 */
public class FeederSubsystem extends SubsystemBase {

    // =====================================================================
    // MOTOR CONTROLLERS
    // =====================================================================
    private final SparkMax indexerMotor;
    private final SparkMax kickerMotor;

    // =====================================================================
    // STATE
    // =====================================================================
    private double indexerTargetRPM = 0;
    private double kickerTargetRPM = 0;

    // =====================================================================
    // TUNABLE PID — INDEXER
    // =====================================================================
    private final TunableNumber indexerKP = new TunableNumber("Feeder/Indexer", "kP", FeederConstants.kIndexerP);
    private final TunableNumber indexerKI = new TunableNumber("Feeder/Indexer", "kI", FeederConstants.kIndexerI);
    private final TunableNumber indexerKD = new TunableNumber("Feeder/Indexer", "kD", FeederConstants.kIndexerD);
    private final TunableNumber indexerKFF = new TunableNumber("Feeder/Indexer", "kFF", FeederConstants.kIndexerFF);

    // =====================================================================
    // TUNABLE PID — KICKER
    // =====================================================================
    private final TunableNumber kickerKP = new TunableNumber("Feeder/Kicker", "kP", FeederConstants.kKickerP);
    private final TunableNumber kickerKI = new TunableNumber("Feeder/Kicker", "kI", FeederConstants.kKickerI);
    private final TunableNumber kickerKD = new TunableNumber("Feeder/Kicker", "kD", FeederConstants.kKickerD);
    private final TunableNumber kickerKFF = new TunableNumber("Feeder/Kicker", "kFF", FeederConstants.kKickerFF);

    // =====================================================================
    // TUNABLE SPEEDS (RPM — Dashboard'dan ayarlanabilir)
    // =====================================================================
    private final TunableNumber tunableIndexerFeedRPM = new TunableNumber("Feeder/Indexer", "Feed RPM",
            FeederConstants.kIndexerFeedRPM);
    private final TunableNumber tunableIndexerSlowRPM = new TunableNumber("Feeder/Indexer", "Slow Feed RPM",
            FeederConstants.kIndexerSlowFeedRPM);
    private final TunableNumber tunableIndexerReverseRPM = new TunableNumber("Feeder/Indexer", "Reverse RPM",
            FeederConstants.kIndexerReverseRPM);

    private final TunableNumber tunableKickerFeedRPM = new TunableNumber("Feeder/Kicker", "Feed RPM",
            FeederConstants.kKickerFeedRPM);
    private final TunableNumber tunableKickerSlowRPM = new TunableNumber("Feeder/Kicker", "Slow Feed RPM",
            FeederConstants.kKickerSlowFeedRPM);
    private final TunableNumber tunableKickerReverseRPM = new TunableNumber("Feeder/Kicker", "Reverse RPM",
            FeederConstants.kKickerReverseRPM);

    // =====================================================================
    // MANUAL OVERRIDE
    // =====================================================================
    private boolean manualOverrideEnabled = false;
    private double manualIndexerRPM = 0;
    private double manualKickerRPM = 0;

    // =====================================================================
    // CONFIGS
    // =====================================================================
    private final SparkMaxConfig indexerConfig = new SparkMaxConfig();
    private final SparkMaxConfig kickerConfig = new SparkMaxConfig();

    // =====================================================================
    // CONSTRUCTOR
    // =====================================================================
    public FeederSubsystem() {
        indexerMotor = new SparkMax(FeederConstants.kIndexerMotorID, MotorType.kBrushless);
        kickerMotor = new SparkMax(FeederConstants.kKickerMotorID, MotorType.kBrushless);

        configureIndexer();
        configureKicker();

        System.out.println("[Feeder] İkili motor yapılandırıldı — Indexer(ID:"
                + FeederConstants.kIndexerMotorID + ") + Kicker(ID:"
                + FeederConstants.kKickerMotorID + ") Velocity PID");
    }

    // =====================================================================
    // MOTOR CONFIGURATION
    // =====================================================================
    private void configureIndexer() {
        indexerConfig.encoder.velocityConversionFactor(1.0);
        indexerConfig.closedLoop
                .p(indexerKP.get())
                .i(indexerKI.get())
                .d(indexerKD.get())
                .velocityFF(indexerKFF.get())
                .outputRange(-1, 1);
        indexerConfig.inverted(false);
        indexerConfig.idleMode(IdleMode.kCoast);
        indexerConfig.smartCurrentLimit(FeederConstants.kCurrentLimit);

        indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureKicker() {
        kickerConfig.encoder.velocityConversionFactor(1.0);
        kickerConfig.closedLoop
                .p(kickerKP.get())
                .i(kickerKI.get())
                .d(kickerKD.get())
                .velocityFF(kickerKFF.get())
                .outputRange(-1, 1);
        kickerConfig.inverted(false);
        kickerConfig.idleMode(IdleMode.kCoast);
        kickerConfig.smartCurrentLimit(FeederConstants.kCurrentLimit);

        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * PID/FF değerleri değiştiyse motorlara uygular.
     */
    private void checkAndApplyTunables() {
        // Indexer PID
        if (indexerKP.hasChanged() || indexerKI.hasChanged()
                || indexerKD.hasChanged() || indexerKFF.hasChanged()) {
            indexerConfig.closedLoop
                    .p(indexerKP.get())
                    .i(indexerKI.get())
                    .d(indexerKD.get())
                    .velocityFF(indexerKFF.get());
            indexerMotor.configure(indexerConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            System.out.println("[Feeder/Indexer] PID Updated: P=" + indexerKP.get()
                    + ", I=" + indexerKI.get() + ", D=" + indexerKD.get()
                    + ", FF=" + indexerKFF.get());
        }

        // Kicker PID
        if (kickerKP.hasChanged() || kickerKI.hasChanged()
                || kickerKD.hasChanged() || kickerKFF.hasChanged()) {
            kickerConfig.closedLoop
                    .p(kickerKP.get())
                    .i(kickerKI.get())
                    .d(kickerKD.get())
                    .velocityFF(kickerKFF.get());
            kickerMotor.configure(kickerConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            System.out.println("[Feeder/Kicker] PID Updated: P=" + kickerKP.get()
                    + ", I=" + kickerKI.get() + ", D=" + kickerKD.get()
                    + ", FF=" + kickerKFF.get());
        }
    }

    // =====================================================================
    // PERIODIC
    // =====================================================================
    @Override
    public void periodic() {
        checkAndApplyTunables();
        logTelemetry();
    }

    // =====================================================================
    // VELOCITY CONTROL — BOTH MOTORS
    // =====================================================================

    /**
     * Her iki motoru velocity PID ile çalıştırır.
     */
    public void setVelocity(double indexerRPM, double kickerRPM) {
        indexerTargetRPM = indexerRPM;
        kickerTargetRPM = kickerRPM;
        indexerMotor.getClosedLoopController().setSetpoint(indexerRPM, ControlType.kVelocity);
        kickerMotor.getClosedLoopController().setSetpoint(kickerRPM, ControlType.kVelocity);
    }

    /**
     * Indexer velocity PID.
     */
    public void setIndexerVelocity(double rpm) {
        indexerTargetRPM = rpm;
        indexerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * Kicker velocity PID.
     */
    public void setKickerVelocity(double rpm) {
        kickerTargetRPM = rpm;
        kickerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    // =====================================================================
    // HIGH-LEVEL COMMANDS (API uyumluluğu korunuyor)
    // =====================================================================

    /**
     * İleri besleme — her iki motor tunable RPM'de çalışır.
     */
    public void feed() {
        if (!manualOverrideEnabled) {
            setVelocity(tunableIndexerFeedRPM.get(), tunableKickerFeedRPM.get());
        }
    }

    /**
     * Yavaş besleme — her iki motor tunable yavaş RPM'de çalışır.
     */
    public void feedSlow() {
        if (!manualOverrideEnabled) {
            setVelocity(tunableIndexerSlowRPM.get(), tunableKickerSlowRPM.get());
        }
    }

    /**
     * Geri çıkarma — her iki motor ters çalışır.
     */
    public void reverse() {
        if (!manualOverrideEnabled) {
            setVelocity(tunableIndexerReverseRPM.get(), tunableKickerReverseRPM.get());
        }
    }

    /**
     * Durdur — her iki motor durur.
     */
    public void stop() {
        if (!manualOverrideEnabled) {
            indexerTargetRPM = 0;
            kickerTargetRPM = 0;
            indexerMotor.stopMotor();
            kickerMotor.stopMotor();
        }
    }

    // =====================================================================
    // INDIVIDUAL MOTOR COMMANDS
    // =====================================================================

    public void feedIndexer() {
        setIndexerVelocity(tunableIndexerFeedRPM.get());
    }

    public void feedKicker() {
        setKickerVelocity(tunableKickerFeedRPM.get());
    }

    public void stopIndexer() {
        indexerTargetRPM = 0;
        indexerMotor.stopMotor();
    }

    public void stopKicker() {
        kickerTargetRPM = 0;
        kickerMotor.stopMotor();
    }

    // =====================================================================
    // MANUAL OVERRIDE
    // =====================================================================

    public void enableManualOverride() {
        manualOverrideEnabled = true;
        System.out.println("[Feeder] Manual override ENABLED");
    }

    public void disableManualOverride() {
        manualOverrideEnabled = false;
        manualIndexerRPM = 0;
        manualKickerRPM = 0;
        indexerMotor.stopMotor();
        kickerMotor.stopMotor();
        System.out.println("[Feeder] Manual override DISABLED");
    }

    public boolean isManualOverrideEnabled() {
        return manualOverrideEnabled;
    }

    /**
     * Manuel override — Indexer RPM ayarla.
     */
    public void setManualIndexerRPM(double rpm) {
        if (manualOverrideEnabled) {
            manualIndexerRPM = rpm;
            indexerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        }
    }

    /**
     * Manuel override — Kicker RPM ayarla.
     */
    public void setManualKickerRPM(double rpm) {
        if (manualOverrideEnabled) {
            manualKickerRPM = rpm;
            kickerMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
        }
    }

    /**
     * Manuel override — her iki motoru RPM ile çalıştır (legacy uyumluluk).
     */
    public void setManualOverrideRPM(double rpm) {
        setManualIndexerRPM(rpm);
        setManualKickerRPM(rpm);
    }

    /**
     * Manuel override — ileri çalıştır.
     */
    public void manualFeed() {
        if (manualOverrideEnabled) {
            manualIndexerRPM = tunableIndexerFeedRPM.get();
            manualKickerRPM = tunableKickerFeedRPM.get();
            indexerMotor.getClosedLoopController().setSetpoint(manualIndexerRPM, ControlType.kVelocity);
            kickerMotor.getClosedLoopController().setSetpoint(manualKickerRPM, ControlType.kVelocity);
        }
    }

    /**
     * Manuel override — durdur.
     */
    public void manualStop() {
        if (manualOverrideEnabled) {
            manualIndexerRPM = 0;
            manualKickerRPM = 0;
        }
        indexerMotor.stopMotor();
        kickerMotor.stopMotor();
    }

    // =====================================================================
    // STATUS
    // =====================================================================

    /** Indexer mevcut hızı (RPM). */
    public double getIndexerVelocityRPM() {
        return indexerMotor.getEncoder().getVelocity();
    }

    /** Kicker mevcut hızı (RPM). */
    public double getKickerVelocityRPM() {
        return kickerMotor.getEncoder().getVelocity();
    }

    /** Uyumluluk: Indexer velocity döndürür. */
    public double getVelocityRPM() {
        return getIndexerVelocityRPM();
    }

    /** Her iki motor da hedefe ulaştı mı? */
    public boolean isAtTarget() {
        boolean indexerOk = Math.abs(getIndexerVelocityRPM() - indexerTargetRPM) < FeederConstants.kRPMTolerance;
        boolean kickerOk = Math.abs(getKickerVelocityRPM() - kickerTargetRPM) < FeederConstants.kRPMTolerance;
        return indexerOk && kickerOk;
    }

    /** Feeder çalışıyor mu? (herhangi bir motor) */
    public boolean isRunning() {
        return Math.abs(getIndexerVelocityRPM()) > 50 || Math.abs(getKickerVelocityRPM()) > 50;
    }

    // =====================================================================
    // TELEMETRY
    // =====================================================================
    private void logTelemetry() {
        Logger.recordOutput("Tuning/Feeder/Indexer/TargetRPM", indexerTargetRPM);
        Logger.recordOutput("Tuning/Feeder/Indexer/ActualRPM", getIndexerVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/Kicker/TargetRPM", kickerTargetRPM);
        Logger.recordOutput("Tuning/Feeder/Kicker/ActualRPM", getKickerVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/ManualOverrideEnabled", manualOverrideEnabled);
        Logger.recordOutput("Tuning/Feeder/ManualIndexerRPM", manualIndexerRPM);
        Logger.recordOutput("Tuning/Feeder/ManualKickerRPM", manualKickerRPM);
    }

    // =====================================================================
    // COMPATIBILITY METHODS
    // =====================================================================

    public double getTunableFeedVoltage() {
        return tunableIndexerFeedRPM.get();
    }

    public double getTunableReverseVoltage() {
        return tunableIndexerReverseRPM.get();
    }

    /**
     * Legacy voltaj kontrolü — velocity PID olarak çalıştırır.
     */
    public void setVoltage(double volts) {
        indexerMotor.setVoltage(volts);
        kickerMotor.setVoltage(volts);
    }
}

package frc.robot.subsystems.lift;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.LiftConstants;

/**
 * Birleşik Lift alt sistemi (Elevator + Climber).
 * 
 * <p>
 * Robot üzerindeki dikey hareket mekanizmasını kontrol eder.
 * Hem skor pozisyonları hem de tırmanma için kullanılır.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Level tabanlı pozisyonlama (L0-L3, Intake)</li>
 * <li>Climb pozisyonları (Extend, Retract, Hold)</li>
 * <li>Motion Magic ile yumuşak hareket</li>
 * <li>Gyro tabanlı eğim kontrolü</li>
 * <li>CAN mesaj optimizasyonu</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Sol Motor: Kraken X60 (CAN ID: 20)</li>
 * <li>Sağ Motor: Kraken X60 (CAN ID: 23)</li>
 * </ul>
 * 
 * @author FRC Team
 * @see LiftIO
 * @see LiftConstants
 */
public class LiftSubsystem extends SubsystemBase {

    /**
     * Lift pozisyon presetleri.
     */
    public enum LiftPreset {
        HOME(LiftConstants.kLevel0Position),
        LEVEL1(LiftConstants.kLevel1Position),
        LEVEL2(LiftConstants.kLevel2Position),
        LEVEL3(LiftConstants.kLevel3Position),
        INTAKE(LiftConstants.kIntakePosition),
        CLIMB_EXTEND(LiftConstants.kClimbExtendPosition),
        CLIMB_RETRACT(LiftConstants.kClimbRetractPosition),
        CLIMB_HOLD(LiftConstants.kClimbHoldPosition);

        public final double position;

        LiftPreset(double position) {
            this.position = position;
        }
    }

    private final LiftIO io;
    private final LiftIOInputsAutoLogged inputs = new LiftIOInputsAutoLogged();

    /** Mevcut hedef preset */
    private LiftPreset currentPreset = LiftPreset.HOME;

    /** Gyro pitch verisi için supplier */
    private Supplier<Double> pitchSupplier = () -> 0.0;

    /** Gyro roll verisi için supplier */
    private Supplier<Double> rollSupplier = () -> 0.0;

    /**
     * Yeni bir LiftSubsystem oluşturur.
     * 
     * @param io Lift IO implementasyonu (Kraken, Sim veya Replay)
     */
    public LiftSubsystem(LiftIO io) {
        this.io = io;
    }

    /**
     * Gyro verilerini bağlar.
     * 
     * @param pitchSupplier Pitch açısı sağlayıcı (derece)
     * @param rollSupplier  Roll açısı sağlayıcı (derece)
     */
    public void setGyroSuppliers(Supplier<Double> pitchSupplier, Supplier<Double> rollSupplier) {
        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Lift", inputs);

        // Pozisyon ve durum logları
        double avgPosition = getPosition();
        Logger.recordOutput("Lift/AveragePosition", avgPosition);
        Logger.recordOutput("Lift/CurrentPreset", currentPreset.name());
        Logger.recordOutput("Lift/AtTarget", isAtTarget());

        // Motor durumu
        double totalCurrent = inputs.leftCurrentAmps + inputs.rightCurrentAmps;
        Logger.recordOutput("Lift/TotalCurrent", totalCurrent);
        Logger.recordOutput("Lift/LeftStalling", inputs.leftCurrentAmps > LiftConstants.kStallCurrentThreshold);
        Logger.recordOutput("Lift/RightStalling", inputs.rightCurrentAmps > LiftConstants.kStallCurrentThreshold);

        // Eğim durumu (tırmanma için önemli)
        double pitch = pitchSupplier.get();
        double roll = rollSupplier.get();
        double maxTilt = Math.max(Math.abs(pitch), Math.abs(roll));
        Logger.recordOutput("Lift/Pitch", pitch);
        Logger.recordOutput("Lift/Roll", roll);
        Logger.recordOutput("Lift/MaxTilt", maxTilt);
        Logger.recordOutput("Lift/TiltWarning", maxTilt > LiftConstants.kTiltWarningThreshold);
        Logger.recordOutput("Lift/TiltDanger", maxTilt > LiftConstants.kTiltDangerThreshold);
    }

    // ==================== PRESET KONTROL ====================

    /**
     * Belirtilen preset pozisyonuna hareket eder.
     * 
     * @param preset Hedef preset
     */
    public void goToPreset(LiftPreset preset) {
        currentPreset = preset;
        io.setPosition(preset.position);
    }

    /**
     * Belirtilen level'a hareket eder (geriye uyumluluk).
     * 
     * @param level Level numarası (0-3, 7=intake)
     */
    public void goToLevel(int level) {
        switch (level) {
            case 0:
                goToPreset(LiftPreset.HOME);
                break;
            case 1:
                goToPreset(LiftPreset.LEVEL1);
                break;
            case 2:
                goToPreset(LiftPreset.LEVEL2);
                break;
            case 3:
                goToPreset(LiftPreset.LEVEL3);
                break;
            case 7:
                goToPreset(LiftPreset.INTAKE);
                break;
            default:
                goToPreset(LiftPreset.HOME);
        }
    }

    /**
     * Hedef pozisyona ulaşılıp ulaşılmadığını kontrol eder.
     * 
     * @return {@code true} eğer tolerans içinde ise
     */
    public boolean isAtTarget() {
        double avgPosition = getPosition();
        return Math.abs(avgPosition - currentPreset.position) < LiftConstants.kPositionTolerance;
    }

    // ==================== CLIMB KONTROL ====================

    /**
     * Tırmanma için uzatır.
     */
    public void climbExtend() {
        goToPreset(LiftPreset.CLIMB_EXTEND);
    }

    /**
     * Tırmanma için geri çeker.
     */
    public void climbRetract() {
        goToPreset(LiftPreset.CLIMB_RETRACT);
    }

    /**
     * Tırmanma tutma pozisyonu.
     */
    public void climbHold() {
        goToPreset(LiftPreset.CLIMB_HOLD);
    }

    // ==================== MANUEL KONTROL ====================

    /**
     * Manuel yukarı hareket.
     */
    public void goUp() {
        io.setVelocity(LiftConstants.kManualUpVelocity);
    }

    /**
     * Manuel aşağı hareket.
     */
    public void goDown() {
        io.setVelocity(LiftConstants.kManualDownVelocity);
    }

    /**
     * Durdur.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Voltaj uygular.
     * 
     * @param volts Voltaj (-12 ile +12 arası)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    // ==================== DURUM SORGULAMA ====================

    /**
     * Ortalama pozisyonu döndürür.
     * 
     * @return Pozisyon (rotor rotasyonları)
     */
    public double getPosition() {
        return (inputs.leftPositionRotations + inputs.rightPositionRotations) / 2.0;
    }

    /**
     * Mevcut preset'i döndürür.
     * 
     * @return Mevcut preset
     */
    public LiftPreset getCurrentPreset() {
        return currentPreset;
    }

    /**
     * Herhangi bir motorun zorlanıp zorlanmadığını kontrol eder.
     * 
     * @return {@code true} eğer herhangi bir motor zorlanıyorsa
     */
    public boolean isStalling() {
        return inputs.leftCurrentAmps > LiftConstants.kStallCurrentThreshold ||
                inputs.rightCurrentAmps > LiftConstants.kStallCurrentThreshold;
    }

    /**
     * Toplam akım çekimini döndürür.
     * 
     * @return Toplam akım (Amper)
     */
    public double getTotalCurrent() {
        return inputs.leftCurrentAmps + inputs.rightCurrentAmps;
    }

    /**
     * Üst limite ulaşılıp ulaşılmadığını kontrol eder.
     * 
     * @return {@code true} eğer üst limitte ise
     */
    public boolean isAtForwardLimit() {
        return inputs.atForwardLimit;
    }

    /**
     * Alt limite ulaşılıp ulaşılmadığını kontrol eder.
     * 
     * @return {@code true} eğer alt limitte ise
     */
    public boolean isAtReverseLimit() {
        return inputs.atReverseLimit;
    }

    /**
     * Eğim uyarı seviyesinde mi kontrol eder.
     * 
     * @return {@code true} eğer eğim uyarı eşiğini aşmışsa
     */
    public boolean isTiltWarning() {
        double maxTilt = Math.max(Math.abs(pitchSupplier.get()), Math.abs(rollSupplier.get()));
        return maxTilt > LiftConstants.kTiltWarningThreshold;
    }

    /**
     * Tehlikeli seviyede eğik mi kontrol eder.
     * 
     * @return {@code true} eğer eğim tehlike eşiğini aşmışsa
     */
    public boolean isTiltDanger() {
        double maxTilt = Math.max(Math.abs(pitchSupplier.get()), Math.abs(rollSupplier.get()));
        return maxTilt > LiftConstants.kTiltDangerThreshold;
    }

    /**
     * Encoder pozisyonunu sıfırlar.
     */
    public void resetPosition() {
        io.resetPosition();
        currentPreset = LiftPreset.HOME;
    }
}

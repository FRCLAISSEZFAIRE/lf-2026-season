package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.ClimberConstants;

/**
 * Tırmanma (Climber) alt sistemi.
 * 
 * <p>
 * Robotun Tower'a tırmanmasını sağlayan dikey hareket mekanizması.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Motion Magic ile yumuşak hareket</li>
 * <li>Güvenlik limitleri (Soft limits)</li>
 * <li>Gyro tabanlı eğim uyarısı</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Sol Motor: Kraken X60 (CAN ID: 20)</li>
 * <li>Sağ Motor: Kraken X60 (CAN ID: 23)</li>
 * </ul>
 * 
 * @author FRC Team
 * @see ClimberIO
 * @see ClimberConstants
 */
public class ClimberSubsystem extends SubsystemBase {

    /**
     * Tırmanma pozisyon presetleri.
     */
    public enum ClimberPreset {
        HOME(ClimberConstants.kHomePosition),
        EXTEND(ClimberConstants.kClimbExtendPosition),
        RETRACT(ClimberConstants.kClimbRetractPosition),
        HOLD(ClimberConstants.kClimbHoldPosition);

        public final double position;

        ClimberPreset(double position) {
            this.position = position;
        }
    }

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    /** Mevcut hedef preset */
    private ClimberPreset currentPreset = ClimberPreset.HOME;

    /** Gyro pitch verisi için supplier */
    private Supplier<Double> pitchSupplier = () -> 0.0;

    /** Gyro roll verisi için supplier */
    private Supplier<Double> rollSupplier = () -> 0.0;

    /**
     * Yeni bir ClimberSubsystem oluşturur.
     * 
     * @param io Climber IO implementasyonu (Kraken, Sim veya Replay)
     */
    public ClimberSubsystem(ClimberIO io) {
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
        Logger.processInputs("Climber", inputs);

        // Pozisyon ve durum logları
        double avgPosition = getPosition();
        Logger.recordOutput("Climber/AveragePosition", avgPosition);
        Logger.recordOutput("Climber/CurrentPreset", currentPreset.name());
        Logger.recordOutput("Climber/AtTarget", isAtTarget());

        // Motor durumu
        double totalCurrent = inputs.leftCurrentAmps + inputs.rightCurrentAmps;
        Logger.recordOutput("Climber/TotalCurrent", totalCurrent);
        Logger.recordOutput("Climber/LeftStalling", inputs.leftCurrentAmps > ClimberConstants.kStallCurrentThreshold);
        Logger.recordOutput("Climber/RightStalling", inputs.rightCurrentAmps > ClimberConstants.kStallCurrentThreshold);

        // Eğim durumu (tırmanma için önemli)
        double pitch = pitchSupplier.get();
        double roll = rollSupplier.get();
        double maxTilt = Math.max(Math.abs(pitch), Math.abs(roll));
        Logger.recordOutput("Climber/Pitch", pitch);
        Logger.recordOutput("Climber/Roll", roll);
        Logger.recordOutput("Climber/MaxTilt", maxTilt);
        Logger.recordOutput("Climber/TiltWarning", maxTilt > ClimberConstants.kTiltWarningThreshold);
        Logger.recordOutput("Climber/TiltDanger", maxTilt > ClimberConstants.kTiltDangerThreshold);
        Logger.recordOutput("Climber/IsSeated", isSeated());
    }

    // ==================== KONTROL METOTLARI ====================

    /**
     * Belirtilen preset pozisyonuna hareket eder.
     * 
     * @param preset Hedef preset
     */
    private void goToPreset(ClimberPreset preset) {
        currentPreset = preset;
        io.setPosition(preset.position);
    }

    /**
     * Tırmanma kancalarını yukarı uzatır (Tırmanmaya hazırlık).
     */
    public void extend() {
        goToPreset(ClimberPreset.EXTEND);
    }

    /**
     * Robotu yukarı çeker (Kancaları aşağı çeker).
     */
    public void retract() {
        goToPreset(ClimberPreset.RETRACT);
    }

    /**
     * Tutma pozisyonuna gider.
     */
    public void hold() {
        goToPreset(ClimberPreset.HOLD);
    }
    
    /**
     * Başlangıç (kapalı) pozisyonuna döner.
     */
    public void home() {
        goToPreset(ClimberPreset.HOME);
    }

    /**
     * Manuel yukarı hareket (Hız kontrolü).
     */
    public void manualUp() {
        io.setVelocity(ClimberConstants.kManualUpVelocity);
    }

    /**
     * Manuel aşağı hareket (Hız kontrolü).
     */
    public void manualDown() {
        io.setVelocity(ClimberConstants.kManualDownVelocity);
    }

    /**
     * Durdur.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Voltaj uygular (Test veya acil durum için).
     * 
     * @param volts Voltaj (-12 ile +12 arası)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    // ==================== DURUM SORGULAMA ====================

    /**
     * Hedef pozisyona ulaşılıp ulaşılmadığını kontrol eder.
     * 
     * @return {@code true} eğer tolerans içinde ise
     */
    public boolean isAtTarget() {
        double avgPosition = getPosition();
        return Math.abs(avgPosition - currentPreset.position) < ClimberConstants.kPositionTolerance;
    }

    /**
     * Tırmanma kancasındaki sensörün (Seat Sensor) durumunu döndürür.
     * 
     * @return {@code true} eğer kanca tam oturmuşsa
     */
    public boolean isSeated() {
        return inputs.isSeated;
    }

    /**
     * Ortalama pozisyonu döndürür.
     * 
     * @return Pozisyon (rotor rotasyonları)
     */
    public double getPosition() {
        return (inputs.leftPositionRotations + inputs.rightPositionRotations) / 2.0;
    }

    /**
     * Herhangi bir motorun zorlanıp zorlanmadığını kontrol eder.
     * 
     * @return {@code true} eğer herhangi bir motor zorlanıyorsa
     */
    public boolean isStalling() {
        return inputs.leftCurrentAmps > ClimberConstants.kStallCurrentThreshold ||
                inputs.rightCurrentAmps > ClimberConstants.kStallCurrentThreshold;
    }

    /**
     * Encoder pozisyonunu sıfırlar.
     */
    public void resetPosition() {
        io.resetPosition();
        currentPreset = ClimberPreset.HOME;
    }
}

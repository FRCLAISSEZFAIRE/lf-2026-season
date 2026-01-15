package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.FeederConstants;

/**
 * Besleyici alt sistemi.
 * 
 * <p>
 * Intake'ten alınan oyun parçalarını Shooter'a transfer eden mekanizmadır.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>İleri besleme (Intake → Shooter)</li>
 * <li>Geri çıkarma</li>
 * <li>Voltaj tabanlı kontrol</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Motor: Falcon 500 (TalonFX)</li>
 * <li>CAN ID: {@link frc.robot.constants.RobotMap#kFeederMotorID}</li>
 * </ul>
 * 
 * @author FRC Team
 * @see FeederIO
 * @see FeederConstants
 */
public class FeederSubsystem extends SubsystemBase {

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    /**
     * Yeni bir FeederSubsystem oluşturur.
     * 
     * @param io Besleyici IO implementasyonu (Real, Sim veya Replay)
     */
    public FeederSubsystem(FeederIO io) {
        this.io = io;
    }

    /**
     * Periyodik güncelleme metodu.
     * Her robot döngüsünde (20ms) çağrılır.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }

    // ==================== AKSİYONLAR ====================

    /**
     * İleri besleme başlatır (Intake → Shooter).
     * 
     * <p>
     * {@link FeederConstants#kFeedVoltage} voltaj uygular.
     * </p>
     */
    public void feed() {
        io.setVoltage(FeederConstants.kFeedVoltage);
    }

    /**
     * Geri çıkarma başlatır.
     * 
     * <p>
     * {@link FeederConstants#kReverseVoltage} voltaj uygular.
     * </p>
     */
    public void reverse() {
        io.setVoltage(FeederConstants.kReverseVoltage);
    }

    /**
     * Motora doğrudan voltaj uygular.
     * 
     * @param volts Uygulanacak voltaj (-12 ile +12 arası)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /**
     * Besleyiciyi durdurur.
     */
    public void stop() {
        io.stop();
    }

    // ==================== DURUM SORGULAMA ====================

    /**
     * Motorun dönüp dönmediğini kontrol eder.
     * 
     * @return {@code true} eğer RPM > 50 ise
     */
    public boolean isRunning() {
        return Math.abs(inputs.velocityRPM) > 50;
    }

    /**
     * Mevcut hızı döndürür.
     * 
     * @return Hız (RPM)
     */
    public double getVelocityRPM() {
        return inputs.velocityRPM;
    }

    // ==================== FUEL TANK LOGIC ====================

    private boolean isLoading = false;

    /**
     * Yükleme modunu ayarlar.
     * Yükleme yaparken (Intake -> Feeder), feeder sensörlere bakmaksızın çalışabilir veya durabilir.
     * Shooter çalışırken bu mod aktifse besleme yapılmaz.
     */
    public void setLoading(boolean loading) {
        this.isLoading = loading;
    }

    public boolean isLoading() {
        return isLoading;
    }

    /**
     * Yakıt tankında yakıt (fuel) var mı?
     * En alt sensör (Bottom) görüyorsa yakıt vardır.
     */
    public boolean isFuelSystemEmpty() {
        return !inputs.fuelPresentBottom;
    }

    /**
     * Yakıt seviyesini döndürür (0-4).
     */
    public int getFuelLevel() {
        int level = 0;
        if (inputs.fuelPresentBottom) level++;
        if (inputs.fuelPresentLow) level++;
        if (inputs.fuelPresentHigh) level++;
        if (inputs.fuelPresentTop) level++;
        return level;
    }

    /**
     * Tank tamamen dolu mu?
     * Tüm sensörler (Bottom, Low, High, Top) 'var' okuyorsa dolu demektir.
     */
    public boolean isFuelSystemFull() {
        return inputs.fuelPresentBottom && inputs.fuelPresentLow && 
               inputs.fuelPresentHigh && inputs.fuelPresentTop;
    }
}

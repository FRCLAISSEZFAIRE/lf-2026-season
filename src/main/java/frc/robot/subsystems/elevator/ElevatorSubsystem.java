package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.ElevatorConstants;

/**
 * Asansör alt sistemi.
 * 
 * <p>
 * Robot üzerindeki dikey hareket mekanizmasını kontrol eder.
 * Level tabanlı pozisyon kontrolü ve manuel kontrol desteği sağlar.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Level tabanlı pozisyonlama (L0, L1, L2, L3, Intake)</li>
 * <li>Manuel yukarı/aşağı hareket</li>
 * <li>Limit switch desteği</li>
 * <li>AdvantageKit loglama</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Motor: Kraken X60 (TalonFX)</li>
 * <li>CAN ID: {@link frc.robot.constants.RobotMap#kElevatorMotorID}</li>
 * </ul>
 * 
 * @author FRC Team
 * @see ElevatorIO
 * @see ElevatorConstants
 */
public class ElevatorSubsystem extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    /** Mevcut hedef level (0-3 + 7=intake) */
    private int currentLevel = 0;

    /**
     * Yeni bir ElevatorSubsystem oluşturur.
     * 
     * @param io Asansör IO implementasyonu (Real, Sim veya Replay)
     */
    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    /**
     * Periyodik güncelleme metodu.
     * Her robot döngüsünde (20ms) çağrılır.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // AdvantageScope için ek loglar
        Logger.recordOutput("Elevator/CurrentLevel", currentLevel);
        Logger.recordOutput("Elevator/AtTarget", isAtTarget());
    }

    // ==================== LEVEL KONTROL ====================

    /**
     * Belirtilen level'a hareket eder.
     * 
     * <p>
     * Level değerleri:
     * </p>
     * <ul>
     * <li>0 - Home (en alt pozisyon)</li>
     * <li>1 - Level 1 (düşük skor)</li>
     * <li>2 - Level 2 (orta skor)</li>
     * <li>3 - Level 3 (yüksek skor)</li>
     * <li>7 - Intake pozisyonu</li>
     * </ul>
     * 
     * @param level Hedef level numarası
     */
    public void goToLevel(int level) {
        currentLevel = level;
        double targetPosition = getLevelPosition(level);
        io.setPosition(targetPosition);
    }

    /**
     * Level numarasına karşılık gelen pozisyonu döndürür.
     * 
     * @param level Level numarası
     * @return Pozisyon (rotor rotasyonları)
     */
    private double getLevelPosition(int level) {
        switch (level) {
            case 0:
                return ElevatorConstants.kLevel0Position;
            case 1:
                return ElevatorConstants.kLevel1Position;
            case 2:
                return ElevatorConstants.kLevel2Position;
            case 3:
                return ElevatorConstants.kLevel3Position;
            case 7:
                return ElevatorConstants.kIntakePosition;
            default:
                return ElevatorConstants.kLevel0Position;
        }
    }

    /**
     * Asansörün hedef pozisyona ulaşıp ulaşmadığını kontrol eder.
     * 
     * @return {@code true} eğer tolerans içinde ise
     */
    public boolean isAtTarget() {
        double targetPosition = getLevelPosition(currentLevel);
        return Math.abs(inputs.positionRotations - targetPosition) < ElevatorConstants.kPositionTolerance;
    }

    // ==================== MANUEL KONTROL ====================

    /**
     * Asansörü manuel olarak yukarı hareket ettirir.
     * {@link ElevatorConstants#kManualUpVelocity} hızında çalışır.
     */
    public void goUp() {
        io.setVelocity(ElevatorConstants.kManualUpVelocity);
    }

    /**
     * Asansörü manuel olarak aşağı hareket ettirir.
     * {@link ElevatorConstants#kManualDownVelocity} hızında çalışır.
     */
    public void goDown() {
        io.setVelocity(ElevatorConstants.kManualDownVelocity);
    }

    /**
     * Asansörü durdurur.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Asansöre doğrudan voltaj uygular.
     * 
     * @param volts Uygulanacak voltaj (-12 ile +12 arası)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    // ==================== DURUM SORGULAMA ====================

    /**
     * Mevcut pozisyonu döndürür.
     * 
     * @return Pozisyon (rotor rotasyonları)
     */
    public double getPosition() {
        return inputs.positionRotations;
    }

    /**
     * Mevcut hedef level'ı döndürür.
     * 
     * @return Level numarası (0-3 veya 7)
     */
    public int getCurrentLevel() {
        return currentLevel;
    }

    /**
     * Üst limit switch'e ulaşılıp ulaşılmadığını kontrol eder.
     * 
     * @return {@code true} eğer üst limitte ise
     */
    public boolean isAtForwardLimit() {
        return inputs.atForwardLimit;
    }

    /**
     * Alt limit switch'e ulaşılıp ulaşılmadığını kontrol eder.
     * 
     * @return {@code true} eğer alt limitte ise
     */
    public boolean isAtReverseLimit() {
        return inputs.atReverseLimit;
    }

    /**
     * Encoder pozisyonunu sıfırlar.
     * Genellikle homing işlemi için kullanılır.
     */
    public void resetPosition() {
        io.resetPosition();
        currentLevel = 0;
    }
}

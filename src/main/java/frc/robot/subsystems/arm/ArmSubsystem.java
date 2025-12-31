package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.Constants;
import frc.robot.util.TunableNumber;

/**
 * Kol alt sistemi.
 * 
 * <p>
 * Elevator'a bağlı döner kol mekanizmasını kontrol eder.
 * Açı tabanlı pozisyon kontrolü ve preset pozisyonlar sağlar.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Açı tabanlı pozisyonlama (derece)</li>
 * <li>Preset pozisyonlar (Stowed, Intake, Score L1/L2/L3)</li>
 * <li>Shuffleboard üzerinden PID ayarı</li>
 * <li>Soft limit koruması</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Motor: NEO (SparkMax)</li>
 * <li>Encoder: Through Bore Encoder</li>
 * <li>CAN ID: {@link frc.robot.constants.RobotMap#kArmMotorID}</li>
 * </ul>
 * 
 * @author FRC Team
 * @see ArmIO
 * @see ArmConstants
 */
public class ArmSubsystem extends SubsystemBase {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    /** Hedef açı (derece) */
    private double targetAngle = ArmConstants.kStowedAngle;

    // PID Tuning için Shuffleboard widget'ları
    private final TunableNumber tunableP;
    private final TunableNumber tunableI;
    private final TunableNumber tunableD;

    /**
     * Yeni bir ArmSubsystem oluşturur.
     * 
     * @param io Kol IO implementasyonu (Real, Sim veya Replay)
     */
    public ArmSubsystem(ArmIO io) {
        this.io = io;

        // Shuffleboard'dan PID ayarı için
        tunableP = new TunableNumber("Arm", "P", ArmConstants.kAngleP);
        tunableI = new TunableNumber("Arm", "I", ArmConstants.kAngleI);
        tunableD = new TunableNumber("Arm", "D", ArmConstants.kAngleD);
    }

    /**
     * Periyodik güncelleme metodu.
     * Her robot döngüsünde (20ms) çağrılır.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        // Tuning modunda PID değerlerini güncelle
        if (Constants.tuningMode) {
            if (tunableP.hasChanged() || tunableI.hasChanged() || tunableD.hasChanged()) {
                io.setPID(tunableP.get(), tunableI.get(), tunableD.get());
            }
        }

        // AdvantageScope logları
        Logger.recordOutput("Arm/TargetAngle", targetAngle);
        Logger.recordOutput("Arm/AtTarget", isAtTarget());
    }

    // ==================== AÇI KONTROL ====================

    /**
     * Belirtilen açıya hareket eder.
     * 
     * <p>
     * Açı otomatik olarak limit değerleri arasına sınırlandırılır.
     * </p>
     * 
     * @param angleDegrees Hedef açı (derece)
     */
    public void setAngle(double angleDegrees) {
        targetAngle = MathUtil.clamp(
                angleDegrees,
                ArmConstants.kAngleLowerLimitDegrees,
                ArmConstants.kAngleUpperLimitDegrees);
        io.setAngle(targetAngle);
    }

    /**
     * Önceden tanımlı preset pozisyonlardan birine hareket eder.
     * 
     * @param preset Hedef preset ({@link ArmPreset})
     */
    public void goToPreset(ArmPreset preset) {
        setAngle(preset.angle);
    }

    /**
     * Kolun hedef açıya ulaşıp ulaşmadığını kontrol eder.
     * 
     * @return {@code true} eğer tolerans içinde ise
     */
    public boolean isAtTarget() {
        return Math.abs(inputs.anglePositionDegrees - targetAngle) < ArmConstants.kAngleTolerance;
    }

    // ==================== MANUEL KONTROL ====================

    /**
     * Kola doğrudan voltaj uygular.
     * 
     * @param volts Uygulanacak voltaj
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /**
     * Kolu durdurur.
     */
    public void stop() {
        io.stop();
    }

    // ==================== DURUM SORGULAMA ====================

    /**
     * Mevcut açıyı döndürür.
     * 
     * @return Açı (derece)
     */
    public double getAngle() {
        return inputs.anglePositionDegrees;
    }

    /**
     * Hedef açıyı döndürür.
     * 
     * @return Hedef açı (derece)
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    // ==================== PRESET TANIMLARI ====================

    /**
     * Kol preset pozisyonları.
     * 
     * <p>
     * Her preset, belirli bir görev için optimize edilmiş açı değeri içerir.
     * </p>
     */
    public enum ArmPreset {
        /** Kapalı/taşıma pozisyonu */
        STOWED(ArmConstants.kStowedAngle),
        /** Oyun parçası alma pozisyonu */
        INTAKE(ArmConstants.kIntakeAngle),
        /** Level 1 skor pozisyonu */
        SCORE_L1(ArmConstants.kScoreL1Angle),
        /** Level 2 skor pozisyonu */
        SCORE_L2(ArmConstants.kScoreL2Angle),
        /** Level 3 skor pozisyonu */
        SCORE_L3(ArmConstants.kScoreL3Angle);

        /** Preset açı değeri (derece) */
        public final double angle;

        ArmPreset(double angle) {
            this.angle = angle;
        }
    }
}

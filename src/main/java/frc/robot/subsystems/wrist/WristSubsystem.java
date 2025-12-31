package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.WristConstants;

/**
 * Bilek alt sistemi.
 * 
 * <p>
 * Arm ucunda ince açı ayarı sağlayan döner eklemdir.
 * Oyun parçasının doğru açıyla yerleştirilmesi için kullanılır.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Açı tabanlı pozisyon kontrolü</li>
 * <li>Preset pozisyonlar (Center, Intake, Score)</li>
 * <li>Soft limit koruması</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Motor: NEO 550 (SparkMax)</li>
 * <li>CAN ID: {@link frc.robot.constants.RobotMap#kWristMotorID}</li>
 * </ul>
 * 
 * @author FRC Team
 * @see WristIO
 * @see WristConstants
 */
public class WristSubsystem extends SubsystemBase {

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    /** Hedef açı (derece) */
    private double targetAngle = WristConstants.kCenterAngle;

    /**
     * Yeni bir WristSubsystem oluşturur.
     * 
     * @param io Bilek IO implementasyonu (Real, Sim veya Replay)
     */
    public WristSubsystem(WristIO io) {
        this.io = io;
    }

    /**
     * Periyodik güncelleme metodu.
     * Her robot döngüsünde (20ms) çağrılır.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        Logger.recordOutput("Wrist/TargetAngle", targetAngle);
    }

    // ==================== AKSİYONLAR ====================

    /**
     * Belirtilen açıya hareket eder.
     * 
     * <p>
     * Açı otomatik olarak limit değerleri arasına sınırlandırılır.
     * </p>
     * 
     * @param degrees Hedef açı (derece)
     */
    public void setAngle(double degrees) {
        targetAngle = Math.max(WristConstants.kMinAngle,
                Math.min(WristConstants.kMaxAngle, degrees));
        io.setAngle(targetAngle);
    }

    /**
     * Merkez pozisyonuna (0°) hareket eder.
     */
    public void goToCenter() {
        setAngle(WristConstants.kCenterAngle);
    }

    /**
     * Intake pozisyonuna hareket eder.
     */
    public void goToIntake() {
        setAngle(WristConstants.kIntakeAngle);
    }

    /**
     * Skor pozisyonuna hareket eder.
     */
    public void goToScore() {
        setAngle(WristConstants.kScoreAngle);
    }

    /**
     * Bileği durdurur.
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
        return inputs.positionDegrees;
    }

    /**
     * Bileğin hedef açıya ulaşıp ulaşmadığını kontrol eder.
     * 
     * @return {@code true} eğer tolerans içinde ise
     */
    public boolean isAtTarget() {
        return Math.abs(inputs.positionDegrees - targetAngle) < WristConstants.kAngleTolerance;
    }
}

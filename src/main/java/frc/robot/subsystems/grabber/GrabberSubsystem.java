package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.Constants;
import frc.robot.constants.GrabberConstants;
import frc.robot.util.TunableNumber;

/**
 * Tutucu alt sistemi.
 * 
 * <p>
 * Oyun parçalarını tutmak ve bırakmak için kullanılan mekanizmadır.
 * Açık/kapalı pozisyonlar arasında geçiş yapar.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Açık/kapalı pozisyon kontrolü</li>
 * <li>Toggle (aç/kapa) fonksiyonu</li>
 * <li>Through Bore Encoder ile pozisyon geri bildirimi</li>
 * <li>Shuffleboard üzerinden PID ayarı</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Motor: NEO 550 (SparkMax)</li>
 * <li>Encoder: Through Bore Encoder</li>
 * <li>CAN ID: {@link frc.robot.constants.RobotMap#kGrabberMotorID}</li>
 * </ul>
 * 
 * @author FRC Team
 * @see GrabberIO
 * @see GrabberConstants
 */
public class GrabberSubsystem extends SubsystemBase {

    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    /** Tutucu açık mı? */
    private boolean isOpen = true;

    /** Hedef pozisyon (derece) */
    private double targetPosition = GrabberConstants.kOpenPosition;

    // PID Tuning için Shuffleboard widget'ları
    private final TunableNumber tunableP;
    private final TunableNumber tunableI;
    private final TunableNumber tunableD;

    /**
     * Yeni bir GrabberSubsystem oluşturur.
     * 
     * @param io Tutucu IO implementasyonu (Real, Sim veya Replay)
     */
    public GrabberSubsystem(GrabberIO io) {
        this.io = io;

        // Shuffleboard'dan PID ayarı için
        tunableP = new TunableNumber("Grabber", "P", GrabberConstants.kP);
        tunableI = new TunableNumber("Grabber", "I", GrabberConstants.kI);
        tunableD = new TunableNumber("Grabber", "D", GrabberConstants.kD);
    }

    /**
     * Periyodik güncelleme metodu.
     * Her robot döngüsünde (20ms) çağrılır.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);

        // Tuning modunda PID değerlerini güncelle
        if (Constants.tuningMode) {
            if (tunableP.hasChanged() || tunableI.hasChanged() || tunableD.hasChanged()) {
                io.setPID(tunableP.get(), tunableI.get(), tunableD.get());
            }
        }

        // AdvantageScope logları
        Logger.recordOutput("Grabber/IsOpen", isOpen);
        Logger.recordOutput("Grabber/TargetPosition", targetPosition);
        Logger.recordOutput("Grabber/AtTarget", isAtTarget());
    }

    // ==================== AKSİYONLAR ====================

    /**
     * Tutucuyu açar.
     * 
     * <p>
     * Oyun parçasını bırakmak için kullanılır.
     * </p>
     */
    public void open() {
        isOpen = true;
        targetPosition = GrabberConstants.kOpenPosition;
        io.setPosition(targetPosition);
    }

    /**
     * Tutucuyu kapatır.
     * 
     * <p>
     * Oyun parçasını tutmak için kullanılır.
     * </p>
     */
    public void close() {
        isOpen = false;
        targetPosition = GrabberConstants.kClosedPosition;
        io.setPosition(targetPosition);
    }

    /**
     * Tutucu durumunu değiştirir (toggle).
     * 
     * <p>
     * Açıksa kapatır, kapalıysa açar.
     * </p>
     */
    public void toggle() {
        if (isOpen) {
            close();
        } else {
            open();
        }
    }

    /**
     * Tutucuyu durdurur.
     */
    public void stop() {
        io.stop();
    }

    // ==================== DURUM SORGULAMA ====================

    /**
     * Tutucunun hedef pozisyona ulaşıp ulaşmadığını kontrol eder.
     * 
     * @return {@code true} eğer tolerans içinde ise
     */
    public boolean isAtTarget() {
        return Math.abs(inputs.positionDegrees - targetPosition) < GrabberConstants.kPositionTolerance;
    }

    /**
     * Tutucunun açık olup olmadığını döndürür.
     * 
     * @return {@code true} eğer tutucu açıksa
     */
    public boolean isOpen() {
        return isOpen;
    }

    /**
     * Mevcut pozisyonu döndürür.
     * 
     * @return Pozisyon (derece)
     */
    public double getPosition() {
        return inputs.positionDegrees;
    }
}

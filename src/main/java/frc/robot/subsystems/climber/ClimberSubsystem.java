package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.ClimberConstants;

/**
 * Tırmanıcı alt sistemi.
 * 
 * <p>
 * Robot'un tırmanma mekanizmasını kontrol eder.
 * İki ayrı motor ile simetrik tırmanma sağlar.
 * </p>
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>İki motor senkronize kontrolü</li>
 * <li>Motor zorlanma (stall) tespiti</li>
 * <li>Gyro tabanlı eğim takibi</li>
 * <li>Uyarı ve tehlike seviyeleri</li>
 * </ul>
 * 
 * <h2>Donanım:</h2>
 * <ul>
 * <li>Sol Motor: Kraken X60 (CAN ID:
 * {@link frc.robot.constants.RobotMap#kClimberLeftMotorID})</li>
 * <li>Sağ Motor: Kraken X60 (CAN ID:
 * {@link frc.robot.constants.RobotMap#kClimberRightMotorID})</li>
 * </ul>
 * 
 * <h2>Güvenlik:</h2>
 * <ul>
 * <li>Akım limiti: {@link ClimberConstants#kCurrentLimit} A</li>
 * <li>Zorlanma eşiği: {@link ClimberConstants#kStallCurrentThreshold} A</li>
 * <li>Eğim uyarısı: {@link ClimberConstants#kTiltWarningThreshold}°</li>
 * <li>Eğim tehlikesi: {@link ClimberConstants#kTiltDangerThreshold}°</li>
 * </ul>
 * 
 * @author FRC Team
 * @see ClimberIO
 * @see ClimberConstants
 */
public class ClimberSubsystem extends SubsystemBase {

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    /** Gyro pitch verisi için supplier (DriveSubsystem'den) */
    private Supplier<Double> pitchSupplier = () -> 0.0;

    /** Gyro roll verisi için supplier (DriveSubsystem'den) */
    private Supplier<Double> rollSupplier = () -> 0.0;

    /**
     * Yeni bir ClimberSubsystem oluşturur.
     * 
     * @param io Tırmanıcı IO implementasyonu (Real, Sim veya Replay)
     */
    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
    }

    /**
     * Gyro verilerini bağlar.
     * 
     * <p>
     * RobotContainer'da DriveSubsystem ile bağlantı için kullanılır.
     * </p>
     * 
     * @param pitchSupplier Pitch açısı sağlayıcı (derece)
     * @param rollSupplier  Roll açısı sağlayıcı (derece)
     */
    public void setGyroSuppliers(Supplier<Double> pitchSupplier, Supplier<Double> rollSupplier) {
        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;
    }

    /**
     * Periyodik güncelleme metodu.
     * Her robot döngüsünde (20ms) çağrılır.
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        // Motor zorlanma kontrolü
        boolean leftStalling = inputs.leftCurrentAmps > ClimberConstants.kStallCurrentThreshold;
        boolean rightStalling = inputs.rightCurrentAmps > ClimberConstants.kStallCurrentThreshold;

        // Gyro verileri
        double pitch = pitchSupplier.get();
        double roll = rollSupplier.get();
        double maxTilt = Math.max(Math.abs(pitch), Math.abs(roll));

        // Eğim durumu
        boolean tiltWarning = maxTilt > ClimberConstants.kTiltWarningThreshold;
        boolean tiltDanger = maxTilt > ClimberConstants.kTiltDangerThreshold;

        // AdvantageScope logları - Motor
        Logger.recordOutput("Climber/LeftStalling", leftStalling);
        Logger.recordOutput("Climber/RightStalling", rightStalling);
        Logger.recordOutput("Climber/AnyStalling", leftStalling || rightStalling);
        Logger.recordOutput("Climber/TotalCurrent", inputs.leftCurrentAmps + inputs.rightCurrentAmps);

        // AdvantageScope logları - Eğim
        Logger.recordOutput("Climber/Pitch", pitch);
        Logger.recordOutput("Climber/Roll", roll);
        Logger.recordOutput("Climber/MaxTilt", maxTilt);
        Logger.recordOutput("Climber/TiltWarning", tiltWarning);
        Logger.recordOutput("Climber/TiltDanger", tiltDanger);
    }

    // ==================== AKSİYONLAR ====================

    /**
     * Tırmanma hareketi başlatır (yukarı).
     * 
     * <p>
     * {@link ClimberConstants#kClimbUpVoltage} voltaj uygular.
     * </p>
     */
    public void climbUp() {
        io.setVoltage(ClimberConstants.kClimbUpVoltage);
    }

    /**
     * İniş hareketi başlatır (aşağı).
     * 
     * <p>
     * {@link ClimberConstants#kClimbDownVoltage} voltaj uygular.
     * </p>
     */
    public void climbDown() {
        io.setVoltage(ClimberConstants.kClimbDownVoltage);
    }

    /**
     * Tırmanıcıyı durdurur.
     */
    public void stop() {
        io.stop();
    }

    /**
     * Motorlara doğrudan voltaj uygular.
     * 
     * @param volts Uygulanacak voltaj (-12 ile +12 arası)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    // ==================== DURUM SORGULAMA ====================

    /**
     * Sol motorun zorlanıp zorlanmadığını kontrol eder.
     * 
     * @return {@code true} eğer akım eşiği aşılmışsa
     */
    public boolean isLeftStalling() {
        return inputs.leftCurrentAmps > ClimberConstants.kStallCurrentThreshold;
    }

    /**
     * Sağ motorun zorlanıp zorlanmadığını kontrol eder.
     * 
     * @return {@code true} eğer akım eşiği aşılmışsa
     */
    public boolean isRightStalling() {
        return inputs.rightCurrentAmps > ClimberConstants.kStallCurrentThreshold;
    }

    /**
     * Herhangi bir motorun zorlanıp zorlanmadığını kontrol eder.
     * 
     * @return {@code true} eğer herhangi bir motor zorlanıyorsa
     */
    public boolean isStalling() {
        return isLeftStalling() || isRightStalling();
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
     * Ortalama pozisyonu döndürür.
     * 
     * @return Pozisyon (rotor rotasyonları)
     */
    public double getPosition() {
        return (inputs.leftPositionRotations + inputs.rightPositionRotations) / 2.0;
    }

    /**
     * Robot eğim uyarı seviyesinde mi kontrol eder.
     * 
     * @return {@code true} eğer eğim uyarı eşiğini aşmışsa
     */
    public boolean isTiltWarning() {
        double maxTilt = Math.max(Math.abs(pitchSupplier.get()), Math.abs(rollSupplier.get()));
        return maxTilt > ClimberConstants.kTiltWarningThreshold;
    }

    /**
     * Robot tehlikeli seviyede eğik mi kontrol eder.
     * 
     * @return {@code true} eğer eğim tehlike eşiğini aşmışsa
     */
    public boolean isTiltDanger() {
        double maxTilt = Math.max(Math.abs(pitchSupplier.get()), Math.abs(rollSupplier.get()));
        return maxTilt > ClimberConstants.kTiltDangerThreshold;
    }
}

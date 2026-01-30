package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.FeederConstants;
import frc.robot.constants.RobotMap;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

/**
 * REVLib 2026 tabanlı Feeder alt sistemi.
 * setSetpoint velocity control kullanır.
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>NEO motor + REVLib Onboard Velocity PID</li>
 * <li>MZ-80 yakıt sensörleri</li>
 * <li>Intake → Shooter arası transfer</li>
 * </ul>
 */
public class FeederSubsystem extends SubsystemBase {

    // =====================================================================
    // MOTOR CONTROLLER
    // =====================================================================
    private final SparkMax feederMotor;

    // =====================================================================
    // SENSORS
    // =====================================================================
    private final DigitalInput fuelSensorBottom;
    private final DigitalInput fuelSensorTop;

    // =====================================================================
    // STATE
    // =====================================================================
    private double targetRPM = 0;
    private boolean isLoading = false;

    // =====================================================================
    // TUNABLE PID
    // =====================================================================
    private final TunableNumber kP = new TunableNumber("Feeder", "kP", FeederConstants.kP);
    private final TunableNumber kI = new TunableNumber("Feeder", "kI", FeederConstants.kI);
    private final TunableNumber kD = new TunableNumber("Feeder", "kD", FeederConstants.kD);
    private final TunableNumber kFF = new TunableNumber("Feeder", "kFF", FeederConstants.kFF);

    // =====================================================================
    // CONSTRUCTOR
    // =====================================================================
    public FeederSubsystem() {
        // --- FEEDER MOTOR (NEO + Velocity Control) ---
        feederMotor = new SparkMax(FeederConstants.kFeederMotorID, MotorType.kBrushless);
        configureMotor();

        // --- FUEL SENSORS ---
        fuelSensorBottom = new DigitalInput(RobotMap.kFeederSensorBottomID);
        fuelSensorTop = new DigitalInput(RobotMap.kFeederSensorTopID);

        System.out.println("[Feeder] REVLib 2026 Velocity Control ile yapılandırıldı");
    }

    // =====================================================================
    // MOTOR CONFIGURATION
    // =====================================================================
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();

        // Encoder dönüşüm faktörü (RPM olarak okumak için)
        // NEO encoder: 42 CPR, motor shaft'ta
        config.encoder
                .velocityConversionFactor(1.0); // Motor RPM direkt okunur

        // REVLib Onboard Velocity PID
        config.closedLoop
                .p(kP.get())
                .i(kI.get())
                .d(kD.get())
                .velocityFF(kFF.get())
                .outputRange(-1, 1);

        // Motor ayarları
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(FeederConstants.kCurrentLimit);

        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =====================================================================
    // PERIODIC
    // =====================================================================
    @Override
    public void periodic() {
        logTelemetry();
    }

    // =====================================================================
    // VELOCITY CONTROL (RPM) - setSetpoint API
    // =====================================================================

    /**
     * Feeder hızını ayarlar (RPM).
     * 
     * @param rpm Hedef hız (pozitif = ileri, negatif = geri)
     */
    public void setVelocity(double rpm) {
        targetRPM = rpm;
        feederMotor.getClosedLoopController().setSetpoint(rpm, ControlType.kVelocity);
    }

    /**
     * İleri besleme (varsayılan hızda).
     */
    public void feed() {
        setVelocity(FeederConstants.kFeedRPM);
    }

    /**
     * Yavaş besleme.
     */
    public void feedSlow() {
        setVelocity(FeederConstants.kSlowFeedRPM);
    }

    /**
     * Geri çıkarma.
     */
    public void reverse() {
        setVelocity(FeederConstants.kReverseRPM);
    }

    /**
     * Durdur.
     */
    public void stop() {
        targetRPM = 0;
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
    // FUEL TANK LOGIC
    // =====================================================================

    public void setLoading(boolean loading) {
        this.isLoading = loading;
    }

    public boolean isLoading() {
        return isLoading;
    }

    /**
     * Alt sensör algılıyor mu? (User: False=Empty, True=Full)
     */
    public boolean hasFuelBottom() {
        return fuelSensorBottom.get();
    }

    /**
     * Üst sensör algılıyor mu? (User: False=Empty, True=Full)
     */
    public boolean hasFuelTop() {
        return fuelSensorTop.get();
    }

    /**
     * Yakıt seviyesi (0-2).
     */
    public int getFuelLevel() {
        int level = 0;
        if (hasFuelBottom())
            level++;
        if (hasFuelTop())
            level++;
        return level;
    }

    /**
     * Sistem boş mu?
     */
    public boolean isFuelSystemEmpty() {
        return !hasFuelBottom() && !hasFuelTop();
    }

    /**
     * Sistem dolu mu?
     */
    public boolean isFuelSystemFull() {
        return hasFuelTop();
    }

    // =====================================================================
    // TELEMETRY
    // =====================================================================
    private void logTelemetry() {
        Logger.recordOutput("Feeder/TargetRPM", targetRPM);
        Logger.recordOutput("Feeder/ActualRPM", getVelocityRPM());
        Logger.recordOutput("Feeder/FuelBottom", hasFuelBottom());
        Logger.recordOutput("Feeder/FuelTop", hasFuelTop());
        Logger.recordOutput("Feeder/FuelLevel", getFuelLevel());
        Logger.recordOutput("Feeder/IsLoading", isLoading);

        // Elastic Dashboard Status
        String status = "Unknown";
        if (isFuelSystemEmpty())
            status = "Empty";
        else if (isFuelSystemFull())
            status = "Full";
        else
            status = "Partial";
        Logger.recordOutput("Feeder/Status", status);
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

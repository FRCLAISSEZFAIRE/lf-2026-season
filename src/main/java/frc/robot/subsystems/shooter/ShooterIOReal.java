package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.RelativeEncoder;

import frc.robot.constants.RobotMap;
import frc.robot.constants.ShooterConstants;

/**
 * Shooter için gerçek donanım IO implementasyonu (REVLib 2026).
 * 
 * <h2>Turret Kontrol Yapısı:</h2>
 * <ol>
 * <li>AbsoluteEncoder: Harici REV Through Bore Encoder (başlangıç pozisyonu
 * okuma)</li>
 * <li>PrimaryEncoder (Internal): Closed-loop kontrol için kullanılır</li>
 * <li>Seeding: Robot başlangıcında absolute encoder okunur, motor encoder
 * senkronize edilir</li>
 * </ol>
 * 
 * <h2>Dişli Oranı Hesabı:</h2>
 * 
 * <pre>
 * Motor 1 tur döndüğünde turret (1 / kTurretGearRatio) tur döner
 * positionConversionFactor = 360.0 / kTurretGearRatio (derece/motor tur)
 * </pre>
 */
public class ShooterIOReal implements ShooterIO {

    private final TalonFX flywheelMotor;

    // Turret
    private final SparkMax turretMotor;
    private final SparkAbsoluteEncoder turretAbsoluteEncoder; // Harici - Sadece seeding için
    private final RelativeEncoder turretInternalEncoder; // Dahili - Closed-loop için
    private final SparkClosedLoopController turretController;

    // Hood
    private final SparkMax hoodMotor;
    private final SparkAbsoluteEncoder hoodAbsoluteEncoder;
    private final RelativeEncoder hoodInternalEncoder;
    private final SparkClosedLoopController hoodController;

    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    private boolean turretSeeded = false;
    private boolean hoodSeeded = false;

    public ShooterIOReal() {
        // =========================================================================
        // FLYWHEEL (KRAKEN X60 - Phoenix 6)
        // =========================================================================
        flywheelMotor = new TalonFX(RobotMap.kShooterMasterID);
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();

        flywheelConfig.Slot0.kP = ShooterConstants.kFlywheelP;
        flywheelConfig.Slot0.kI = ShooterConstants.kFlywheelI;
        flywheelConfig.Slot0.kD = ShooterConstants.kFlywheelD;
        flywheelConfig.Slot0.kS = ShooterConstants.kFlywheelkS;
        flywheelConfig.Slot0.kV = ShooterConstants.kFlywheelkV;

        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
        flywheelMotor.optimizeBusUtilization();

        // =========================================================================
        // TURRET (NEO + External Absolute Encoder + Internal Encoder for Control)
        // =========================================================================
        turretMotor = new SparkMax(RobotMap.kTurretMotorID, MotorType.kBrushless);
        turretAbsoluteEncoder = turretMotor.getAbsoluteEncoder(); // Seeding için
        turretInternalEncoder = turretMotor.getEncoder(); // Closed-loop için
        turretController = turretMotor.getClosedLoopController();

        configureTurret(
                ShooterConstants.kTurretDefaultP,
                ShooterConstants.kTurretDefaultI,
                ShooterConstants.kTurretDefaultD);

        // Turret pozisyonunu seed et
        seedTurretPosition();

        // =========================================================================
        // HOOD (NEO 550 + Absolute Encoder + Internal Encoder for Control)
        // =========================================================================
        hoodMotor = new SparkMax(RobotMap.kHoodMotorID, MotorType.kBrushless);
        hoodAbsoluteEncoder = hoodMotor.getAbsoluteEncoder();
        hoodInternalEncoder = hoodMotor.getEncoder();
        hoodController = hoodMotor.getClosedLoopController();

        configureHood(
                ShooterConstants.kHoodP,
                ShooterConstants.kHoodI,
                ShooterConstants.kHoodD);

        // Hood pozisyonunu seed et
        seedHoodPosition();
    }

    /**
     * Turret motor konfigürasyonu.
     * TunableNumber üzerinden PID değiştiğinde bu metod çağrılır.
     */
    public void configureTurret(double p, double i, double d) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40);

        // --- ENCODER BLOĞU ---
        // Dişli oranına göre positionConversionFactor hesapla
        // Motor 1 tur = Turret (1/GearRatio) tur = (360/GearRatio) derece
        double degreesPerMotorRotation = 360.0 / ShooterConstants.kTurretGearRatio;
        config.encoder.positionConversionFactor(degreesPerMotorRotation);
        config.encoder.velocityConversionFactor(degreesPerMotorRotation / 60.0); // derece/saniye

        // --- CLOSED LOOP BLOĞU ---
        // FeedbackSensor.kPrimaryEncoder (Internal) kullan
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.p(p);
        config.closedLoop.i(i);
        config.closedLoop.d(d);

        // Continuous input wrapping (-180 to 180 derece)
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingInputRange(
                ShooterConstants.kTurretWrapMinInput,
                ShooterConstants.kTurretWrapMaxInput);

        // --- SOFT LIMITS ---
        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            config.softLimit.forwardSoftLimit(ShooterConstants.kTurretMaxAngle);
            config.softLimit.reverseSoftLimit(ShooterConstants.kTurretMinAngle);
            config.softLimit.forwardSoftLimitEnabled(true);
            config.softLimit.reverseSoftLimitEnabled(true);
        }

        turretMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Hood motor konfigürasyonu.
     */
    public void configureHood(double p, double i, double d) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(20);

        // --- ENCODER BLOĞU ---
        double degreesPerMotorRotation = 360.0 / ShooterConstants.kHoodGearRatio;
        config.encoder.positionConversionFactor(degreesPerMotorRotation);
        config.encoder.velocityConversionFactor(degreesPerMotorRotation / 60.0);

        // --- CLOSED LOOP BLOĞU ---
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.p(p);
        config.closedLoop.i(i);
        config.closedLoop.d(d);

        hoodMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    /**
     * Turret başlangıç pozisyonunu absolute encoder'dan seed eder.
     * AbsoluteEncoder değerini okur, internal encoder'a yazar.
     */
    public void seedTurretPosition() {
        // Absolute encoder'dan oku (0-1 rotation)
        double absoluteRotations = turretAbsoluteEncoder.getPosition();

        // Offset uygula ve dereceye çevir (absolute encoder 0-360 varsayılan)
        double absoluteDegrees = (absoluteRotations * 360.0) - ShooterConstants.kTurretEncoderOffset;

        // -180 ile 180 arası normalize et
        while (absoluteDegrees > 180.0)
            absoluteDegrees -= 360.0;
        while (absoluteDegrees < -180.0)
            absoluteDegrees += 360.0;

        // Motor internal encoder'ına yaz (seeding)
        turretInternalEncoder.setPosition(absoluteDegrees);

        turretSeeded = true;
        System.out.println("[Turret] Seeded position from absolute encoder: " + absoluteDegrees + "°");
    }

    /**
     * Hood başlangıç pozisyonunu absolute encoder'dan seed eder.
     */
    public void seedHoodPosition() {
        double absoluteRotations = hoodAbsoluteEncoder.getPosition();
        double absoluteDegrees = (absoluteRotations * 360.0) - ShooterConstants.kHoodEncoderOffset;

        // Clamp to valid range
        if (absoluteDegrees < ShooterConstants.kHoodMinAngle)
            absoluteDegrees = ShooterConstants.kHoodMinAngle;
        if (absoluteDegrees > ShooterConstants.kHoodMaxAngle)
            absoluteDegrees = ShooterConstants.kHoodMaxAngle;

        hoodInternalEncoder.setPosition(absoluteDegrees);

        hoodSeeded = true;
        System.out.println("[Hood] Seeded position from absolute encoder: " + absoluteDegrees + "°");
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel
        inputs.flywheelVelocityRadPerSec = flywheelMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelCurrentAmps = flywheelMotor.getTorqueCurrent().getValueAsDouble();

        // Turret - Internal encoder kullan (seeded)
        inputs.turretAbsolutePositionRad = Math.toRadians(turretInternalEncoder.getPosition());
        inputs.turretAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();

        // Hood - Internal encoder kullan (seeded)
        inputs.hoodPositionDegrees = hoodInternalEncoder.getPosition();
        inputs.hoodAppliedVolts = hoodMotor.getAppliedOutput() * hoodMotor.getBusVoltage();

        inputs.shooterSensorTriggered = false;
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setFlywheelVelocity(double velocityRPM) {
        double velocityRPS = velocityRPM / 60.0;
        flywheelMotor.setControl(velocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setTurretPosition(double angleRad) {
        double angleDegrees = Math.toDegrees(angleRad);
        // REVLib 2026: setSetpoint ile internal encoder hedefi
        turretController.setSetpoint(angleDegrees, SparkBase.ControlType.kPosition);
    }

    @Override
    public void setTurretVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        // REVLib 2026: setSetpoint
        hoodController.setSetpoint(angleDegrees, SparkBase.ControlType.kPosition);
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    // Status methods
    public boolean isTurretSeeded() {
        return turretSeeded;
    }

    public boolean isHoodSeeded() {
        return hoodSeeded;
    }

    public void reseedTurret() {
        seedTurretPosition();
    }

    public void reseedHood() {
        seedHoodPosition();
    }
}

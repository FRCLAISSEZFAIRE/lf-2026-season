package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.ShooterConstants;

/**
 * Shooter için gerçek donanım IO implementasyonu.
 * Flywheel (Kraken X60 - Velocity Control), Turret (TalonFX), Hood (Dual Servo)
 */
public class ShooterIOReal implements ShooterIO {

    private final TalonFX flywheelMotor;
    private final TalonFX turretMotor;
    private final Servo leftHoodServo;
    private final Servo rightHoodServo;
    private final DigitalInput mz80Sensor;

    // Kontrol Nesneleri
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public ShooterIOReal(int flywheelID, int turretID, int hoodID) {
        // --- FLYWHEEL (KRAKEN X60) ---
        flywheelMotor = new TalonFX(flywheelID);
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
        
        // PID & FF Ayarları (Slot 0)
        flywheelConfig.Slot0.kP = ShooterConstants.kFlywheelP;
        flywheelConfig.Slot0.kI = ShooterConstants.kFlywheelI;
        flywheelConfig.Slot0.kD = ShooterConstants.kFlywheelD;
        flywheelConfig.Slot0.kS = ShooterConstants.kFlywheelkS;
        flywheelConfig.Slot0.kV = ShooterConstants.kFlywheelkV;
        
        // Genel Ayarlar
        flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        flywheelConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        flywheelMotor.getConfigurator().apply(flywheelConfig);
        
        // CAN Optimizasyonu (Kritik sinyaller hızlı, diğerleri yavaş/kapalı)
        // optimizeBusUtilization(): Sadece gerekli sinyalleri (Status 1, 2) bırakır, diğerlerini kapatır.
        flywheelMotor.optimizeBusUtilization();

        // --- TURRET ---
        turretMotor = new TalonFX(turretID);
        // Turret konfigürasyonu varsayılan veya ayrı yapılabilir...
        
        // --- HOOD (DUAL SERVO) ---
        // 'hoodID' argümanı SparkMAX içindi, artık kullanılmayacak (veya PWM port olarak düşünülebilir ama Constants kullanacağız)
        leftHoodServo = new Servo(ShooterConstants.kHoodLeftPWMPort);
        rightHoodServo = new Servo(ShooterConstants.kHoodRightPWMPort);

        // --- SENSORS ---
        mz80Sensor = new DigitalInput(MechanismConstants.kShooterMZ80Port);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel
        inputs.flywheelVelocityRadPerSec = flywheelMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelCurrentAmps = flywheelMotor.getTorqueCurrent().getValueAsDouble();
        
        // Error Hesaplama (Target - Actual) -> Bunu Subsystem yapabilir ama IO'da raw fark loglayabiliriz
        // Şimdilik 0 bırakıyoruz veya ClosedLoopError sinyali varsa okuyabiliriz:
        // inputs.flywheelVelocityError = flywheelMotor.getClosedLoopError().getValueAsDouble(); // Opsiyonel

        // Turret
        inputs.turretAbsolutePositionRad = turretMotor.getPosition().getValueAsDouble() * 2 * Math.PI;
        inputs.turretAppliedVolts = turretMotor.getMotorVoltage().getValueAsDouble();

        // Hood (Servo geri bildirim vermez, son komutu varsayabiliriz)
        inputs.hoodPositionDegrees = leftHoodServo.getAngle(); // 0-180 döner
        inputs.hoodAppliedVolts = leftHoodServo.get() * 5.0; // Tahmini

        // MZ80 Sensör (active-low)
        inputs.shooterSensorTriggered = !mz80Sensor.get();
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setFlywheelVelocity(double velocityRPM) {
        // RPS'e çevir
        double velocityRPS = velocityRPM / 60.0;
        flywheelMotor.setControl(velocityControl.withVelocity(velocityRPS));
    }

    @Override
    public void setTurretVoltage(double volts) {
        turretMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        // Servo 0-180 derece çalışır (varsayılan)
        // Eğer map gerekirse: (angle - min) / (max - min) --> setPosition(0..1)
        // WPI Servo library setAngle(degrees) metoduna sahiptir ve bunu PWM aralığına map eder.
        
        double leftTarget = ShooterConstants.kHoodLeftServoInverted ? (180.0 - angleDegrees) : angleDegrees;
        double rightTarget = ShooterConstants.kHoodRightServoInverted ? (180.0 - angleDegrees) : angleDegrees;

        leftHoodServo.setAngle(leftTarget);
        rightHoodServo.setAngle(rightTarget);
    }
}

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
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.MechanismConstants;
import frc.robot.constants.ShooterConstants;

/**
 * Shooter için gerçek donanım IO implementasyonu.
 * Flywheel (Kraken X60 - Velocity Control), Turret (NEO - SparkMax), Hood (Dual
 * Servo)
 */
public class ShooterIOReal implements ShooterIO {

    private final TalonFX flywheelMotor;

    private final SparkMax turretMotor;
    private final SparkAbsoluteEncoder turretEncoder;
    private final SparkClosedLoopController turretController;

    private final Servo leftHoodServo;
    private final Servo rightHoodServo;
    // private final DigitalInput mz80Sensor;

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
        flywheelMotor.optimizeBusUtilization();

        // --- TURRET (NEO) ---
        turretMotor = new SparkMax(turretID, MotorType.kBrushless);
        turretEncoder = turretMotor.getAbsoluteEncoder();
        turretController = turretMotor.getClosedLoopController();

        SparkMaxConfig turretConfig = new SparkMaxConfig();
        turretConfig.idleMode(IdleMode.kBrake);
        turretConfig.smartCurrentLimit(30);

        // Turret PID & Absolute Encoder
        turretConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turretConfig.closedLoop.pid(1.0, 0.0, 0.0); // P=1.0 Placeholder

        // Unbounded turret or logical limits? Usually limits.
        // Konfigüre edilebilir ama şimdilik soft limit koymuyoruz yazılımsal
        // halledilir.

        turretConfig.absoluteEncoder.positionConversionFactor(2 * Math.PI); // Radyan
        turretConfig.absoluteEncoder.velocityConversionFactor(2 * Math.PI / 60.0);

        turretMotor.configure(turretConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);

        // --- HOOD (DUAL SERVO) ---
        leftHoodServo = new Servo(ShooterConstants.kHoodLeftPWMPort);
        rightHoodServo = new Servo(ShooterConstants.kHoodRightPWMPort);

        // --- SENSORS ---
        // mz80Sensor = new DigitalInput(MechanismConstants.kShooterMZ80Port); // İptal
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel
        inputs.flywheelVelocityRadPerSec = flywheelMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.flywheelAppliedVolts = flywheelMotor.getMotorVoltage().getValueAsDouble();
        inputs.flywheelCurrentAmps = flywheelMotor.getTorqueCurrent().getValueAsDouble();

        // Turret (SparkMax)
        inputs.turretAbsolutePositionRad = turretEncoder.getPosition();
        inputs.turretAppliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();

        // Hood
        inputs.hoodPositionDegrees = leftHoodServo.getAngle();
        inputs.hoodAppliedVolts = leftHoodServo.get() * 5.0;

        // MZ80 Sensör
        inputs.shooterSensorTriggered = false; // İptal
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
    public void setTurretVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        double leftTarget = ShooterConstants.kHoodLeftServoInverted ? (180.0 - angleDegrees) : angleDegrees;
        double rightTarget = ShooterConstants.kHoodRightServoInverted ? (180.0 - angleDegrees) : angleDegrees;

        leftHoodServo.setAngle(leftTarget);
        rightHoodServo.setAngle(rightTarget);
    }
}

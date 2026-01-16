package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.constants.ClimberConstants;

/**
 * Climber için gerçek donanım IO implementasyonu.
 * 2 Kraken X60 motor ile Motion Magic kontrolü.
 * CAN mesajları optimize edilmiştir.
 */
public class ClimberIOKraken implements ClimberIO {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    // Seat Sensor (Limit Switch)
    private final DigitalInput seatSensor;

    // Control requests (reusable)
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // Cached status signals for reduced CAN usage
    private final StatusSignal<?> leftPosition;
    private final StatusSignal<?> leftVelocity;
    private final StatusSignal<?> leftCurrent;
    private final StatusSignal<?> leftTemp;
    private final StatusSignal<?> leftVoltage;

    private final StatusSignal<?> rightPosition;
    private final StatusSignal<?> rightVelocity;
    private final StatusSignal<?> rightCurrent;
    private final StatusSignal<?> rightTemp;
    private final StatusSignal<?> rightVoltage;

    public ClimberIOKraken(int leftID, int rightID) {
        leftMotor = new TalonFX(leftID);
        rightMotor = new TalonFX(rightID);

        // Seat Sensor init
        seatSensor = new DigitalInput(ClimberConstants.kSeatSensorDIO);

        // Motor konfigürasyonu
        configureMotor(leftMotor, false);
        configureMotor(rightMotor, true); // Sağ motor ters

        // Status signal'ları cache'le
        leftPosition = leftMotor.getPosition();
        leftVelocity = leftMotor.getVelocity();
        leftCurrent = leftMotor.getStatorCurrent();
        leftTemp = leftMotor.getDeviceTemp();
        leftVoltage = leftMotor.getMotorVoltage();

        rightPosition = rightMotor.getPosition();
        rightVelocity = rightMotor.getVelocity();
        rightCurrent = rightMotor.getStatorCurrent();
        rightTemp = rightMotor.getDeviceTemp();
        rightVoltage = rightMotor.getMotorVoltage();

        // CAN mesajlarını optimize et
        optimizeCANUsage();
    }

    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Feedback - Rotor Sensor
        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback = feedback;

        // Soft Limits
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = ClimberConstants.kForwardSoftLimit;
        softLimits.ReverseSoftLimitThreshold = ClimberConstants.kReverseSoftLimit;
        config.SoftwareLimitSwitch = softLimits;

        // Motor Output - Brake mode
        MotorOutputConfigs output = new MotorOutputConfigs();
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput = output;

        // Current Limits
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = ClimberConstants.kSupplyCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        // PID Slot 0 (Motion Magic için)
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = ClimberConstants.kClimberS;
        slot0.kV = ClimberConstants.kClimberV;
        slot0.kA = ClimberConstants.kClimberA;
        slot0.kP = ClimberConstants.kClimberP;
        slot0.kI = ClimberConstants.kClimberI;
        slot0.kD = ClimberConstants.kClimberD;

        // Motion Magic Profiling
        config.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.kClimberMMCV;
        config.MotionMagic.MotionMagicAcceleration = ClimberConstants.kClimberMMA;
        config.MotionMagic.MotionMagicJerk = ClimberConstants.kClimberMMJ;

        motor.getConfigurator().apply(config);
    }

    /**
     * CAN bus kullanımını optimize eder.
     * Update frekanslarını düşürür ve gereksiz sinyalleri devre dışı bırakır.
     */
    private void optimizeCANUsage() {
        // Sol motor sinyalleri
        leftPosition.setUpdateFrequency(ClimberConstants.kPositionUpdateHz);
        leftVelocity.setUpdateFrequency(ClimberConstants.kVelocityUpdateHz);
        leftCurrent.setUpdateFrequency(ClimberConstants.kCurrentUpdateHz);
        leftTemp.setUpdateFrequency(ClimberConstants.kTemperatureUpdateHz);
        leftVoltage.setUpdateFrequency(ClimberConstants.kCurrentUpdateHz);

        // Sağ motor sinyalleri
        rightPosition.setUpdateFrequency(ClimberConstants.kPositionUpdateHz);
        rightVelocity.setUpdateFrequency(ClimberConstants.kVelocityUpdateHz);
        rightCurrent.setUpdateFrequency(ClimberConstants.kCurrentUpdateHz);
        rightTemp.setUpdateFrequency(ClimberConstants.kTemperatureUpdateHz);
        rightVoltage.setUpdateFrequency(ClimberConstants.kCurrentUpdateHz);

        // Gereksiz sinyalleri kapat
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Tüm sinyalleri tek seferde refresh et (daha verimli)
        BaseStatusSignal.refreshAll(
                leftPosition, leftVelocity, leftCurrent, leftTemp, leftVoltage,
                rightPosition, rightVelocity, rightCurrent, rightTemp, rightVoltage);

        // Sol motor
        inputs.leftPositionRotations = leftPosition.getValueAsDouble();
        inputs.leftVelocityRPS = leftVelocity.getValueAsDouble();
        inputs.leftCurrentAmps = leftCurrent.getValueAsDouble();
        inputs.leftTemperatureCelsius = leftTemp.getValueAsDouble();
        inputs.leftAppliedVolts = leftVoltage.getValueAsDouble();

        // Sağ motor
        inputs.rightPositionRotations = rightPosition.getValueAsDouble();
        inputs.rightVelocityRPS = rightVelocity.getValueAsDouble();
        inputs.rightCurrentAmps = rightCurrent.getValueAsDouble();
        inputs.rightTemperatureCelsius = rightTemp.getValueAsDouble();
        inputs.rightAppliedVolts = rightVoltage.getValueAsDouble();

        // Limit kontrolü
        double avgPosition = (inputs.leftPositionRotations + inputs.rightPositionRotations) / 2.0;
        inputs.atForwardLimit = avgPosition >= ClimberConstants.kForwardSoftLimit - 0.5;
        inputs.atReverseLimit = avgPosition <= ClimberConstants.kReverseSoftLimit + 0.5;

        // Seat Sensor (!get() çünkü switch basılıyken genelde false/low döner, ama NO/NC'ye göre değişir.
        // Genelde basıldığında devre tamamlanır (TRUE) ya da kesilir (FALSE).
        // Varsayım: Switch basıldığında TRUE dönüyor. Eğer ters ise !seatSensor.get() yapın.
        // FRC standartlarında limit switchler genelde Normally Open (NO) kullanılır ve basınca 1 olur.
        inputs.isSeated = seatSensor.get();
    }

    @Override
    public void setPosition(double positionRotations) {
        // Her iki motoru da aynı pozisyona gönder
        leftMotor.setControl(positionRequest.withPosition(positionRotations));
        rightMotor.setControl(positionRequest.withPosition(positionRotations));
    }

    @Override
    public void setVelocity(double velocityRPS) {
        leftMotor.setControl(velocityRequest.withVelocity(velocityRPS));
        rightMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    @Override
    public void setVoltage(double volts) {
        leftMotor.setControl(voltageRequest.withOutput(volts));
        rightMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        leftMotor.setControl(voltageRequest.withOutput(0));
        rightMotor.setControl(voltageRequest.withOutput(0));
    }

    @Override
    public void resetPosition() {
        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
    }
}

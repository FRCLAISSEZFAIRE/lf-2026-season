package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.ElevatorConstants;

/**
 * Elevator için gerçek donanım IO implementasyonu.
 * TalonFX ile Motion Magic kontrolü.
 */
public class ElevatorIOReal implements ElevatorIO {

    private final TalonFX motor;

    // Control requests
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public ElevatorIOReal(int motorID) {
        motor = new TalonFX(motorID);
        configureMotor();
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Feedback - Rotor Sensor kullan
        FeedbackConfigs feedback = new FeedbackConfigs();
        feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback = feedback;

        // Soft Limits
        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs();
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ReverseSoftLimitEnable = true;
        softLimits.ForwardSoftLimitThreshold = ElevatorConstants.kForwardSoftLimit;
        softLimits.ReverseSoftLimitThreshold = ElevatorConstants.kReverseSoftLimit;
        config.SoftwareLimitSwitch = softLimits;

        // Motor Output - Brake mode
        MotorOutputConfigs output = new MotorOutputConfigs();
        output.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput = output;

        // PID Slot 0 (Motion Magic için)
        Slot0Configs slot0 = config.Slot0;
        slot0.kS = ElevatorConstants.kElevatorS;
        slot0.kV = ElevatorConstants.kElevatorV;
        slot0.kA = ElevatorConstants.kElevatorA;
        slot0.kP = ElevatorConstants.kElevatorP;
        slot0.kI = ElevatorConstants.kElevatorI;
        slot0.kD = ElevatorConstants.kElevatorD;

        // Motion Magic Profiling
        config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kElevatorMMCV;
        config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kElevatorMMA;
        config.MotionMagic.MotionMagicJerk = ElevatorConstants.kElevatorMMJ;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionRotations = motor.getPosition().getValueAsDouble();
        inputs.velocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();

        // Soft limit kontrolü
        inputs.atForwardLimit = inputs.positionRotations >= ElevatorConstants.kForwardSoftLimit - 0.5;
        inputs.atReverseLimit = inputs.positionRotations <= ElevatorConstants.kReverseSoftLimit + 0.5;
    }

    @Override
    public void setPosition(double positionRotations) {
        motor.setControl(positionRequest.withPosition(positionRotations));
    }

    @Override
    public void setVelocity(double velocityRPS) {
        motor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.setControl(velocityRequest.withVelocity(0));
    }

    @Override
    public void resetPosition() {
        motor.setPosition(0);
    }
}

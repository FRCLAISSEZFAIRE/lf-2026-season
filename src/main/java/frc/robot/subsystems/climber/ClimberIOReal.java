package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.ClimberConstants;

/**
 * Climber için gerçek donanım IO implementasyonu.
 * İki Kraken motor birlikte çalışır.
 */
public class ClimberIOReal implements ClimberIO {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ClimberIOReal(int leftID, int rightID) {
        leftMotor = new TalonFX(leftID);
        rightMotor = new TalonFX(rightID);

        configureMotor(leftMotor, false);
        configureMotor(rightMotor, true); // Sağ motor ters
    }

    private void configureMotor(TalonFX motor, boolean inverted) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Brake mode
        MotorOutputConfigs output = new MotorOutputConfigs();
        output.NeutralMode = NeutralModeValue.Brake;
        output.Inverted = inverted ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        config.MotorOutput = output;

        // Current limits
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = ClimberConstants.kCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits = currentLimits;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Sol motor
        inputs.leftPositionRotations = leftMotor.getPosition().getValueAsDouble();
        inputs.leftVelocityRPS = leftMotor.getVelocity().getValueAsDouble();
        inputs.leftAppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
        inputs.leftTemperatureCelsius = leftMotor.getDeviceTemp().getValueAsDouble();

        // Sağ motor
        inputs.rightPositionRotations = rightMotor.getPosition().getValueAsDouble();
        inputs.rightVelocityRPS = rightMotor.getVelocity().getValueAsDouble();
        inputs.rightAppliedVolts = rightMotor.getMotorVoltage().getValueAsDouble();
        inputs.rightCurrentAmps = rightMotor.getStatorCurrent().getValueAsDouble();
        inputs.rightTemperatureCelsius = rightMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        leftMotor.setControl(voltageControl.withOutput(volts));
        rightMotor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        leftMotor.setControl(voltageControl.withOutput(0));
        rightMotor.setControl(voltageControl.withOutput(0));
    }
}

package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.FeederConstants;

/**
 * Feeder IO gerçek implementasyonu.
 * Falcon 500 (TalonFX) kullanır.
 */
public class FeederIOReal implements FeederIO {

    private final TalonFX motor;
    private final VoltageOut voltageControl = new VoltageOut(0);

    public FeederIOReal(int motorID) {
        motor = new TalonFX(motorID);

        // Konfigürasyon
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = FeederConstants.kCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocityRPM = motor.getVelocity().getValueAsDouble() * 60; // RPS → RPM
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}

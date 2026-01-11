package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.constants.FeederConstants;

/**
 * Feeder IO gerçek implementasyonu.
 * NEO v1.1 (SparkMax) kullanır.
 */
public class FeederIOReal implements FeederIO {

    private final SparkMax motor;

    public FeederIOReal(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);

        // Konfigürasyon
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(FeederConstants.kCurrentLimit);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocityRPM = motor.getEncoder().getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}

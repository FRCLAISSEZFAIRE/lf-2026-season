package frc.robot.subsystems.feeder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;

import frc.robot.constants.FeederConstants;

/**
 * Feeder IO gerçek implementasyonu (REVLib 2026).
 * NEO (SparkMax) kullanır - Voltage control.
 */
public class FeederIOReal implements FeederIO {

    private final SparkMax motor;
    private final edu.wpi.first.wpilibj.DigitalInput sensorBottom;
    private final edu.wpi.first.wpilibj.DigitalInput sensorTop;

    public FeederIOReal(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);

        // REVLib 2026: SparkMaxConfig ile configure
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        config.smartCurrentLimit(FeederConstants.kCurrentLimit);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // Sensörler
        sensorBottom = new edu.wpi.first.wpilibj.DigitalInput(FeederConstants.kFuelSensorBottomID);
        sensorTop = new edu.wpi.first.wpilibj.DigitalInput(FeederConstants.kFuelSensorTopID);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocityRPM = motor.getEncoder().getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();

        // Sensörler (NPN: 0V=Var, 5V=Yok)
        inputs.fuelPresentBottom = !sensorBottom.get();
        inputs.fuelPresentTop = !sensorTop.get();
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

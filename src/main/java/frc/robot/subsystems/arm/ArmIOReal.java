package frc.robot.subsystems.arm;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.ArmConstants;

/**
 * Arm için gerçek donanım IO implementasyonu.
 * SparkMax + AbsoluteEncoder ile açı kontrolü.
 */
public class ArmIOReal implements ArmIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController closedLoopController;

    public ArmIOReal(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
        config = new SparkMaxConfig();

        // Brake mode
        config.idleMode(IdleMode.kBrake);

        // PID
        config.closedLoop.pid(
                ArmConstants.kAngleP,
                ArmConstants.kAngleI,
                ArmConstants.kAngleD);

        // Position wrapping (REVLib 2026 beta için)
        config.closedLoop.positionWrappingEnabled(true);

        // Absolute encoder config
        config.absoluteEncoder.positionConversionFactor(ArmConstants.kAngleEncoderPositionFactor);
        config.absoluteEncoder.inverted(false);

        // Output limits
        config.closedLoop.maxOutput(ArmConstants.kMaxOutput);
        config.closedLoop.minOutput(ArmConstants.kMinOutput);

        // Apply config
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = motor.getAbsoluteEncoder();
        closedLoopController = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.anglePositionDegrees = absoluteEncoder.getPosition();
        inputs.angleVelocityDegPerSec = absoluteEncoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setAngle(double angleDegrees) {
        closedLoopController.setReference(angleDegrees, ControlType.kPosition);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setPID(double p, double i, double d) {
        config.closedLoop.pid(p, i, d);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}

package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.constants.GrabberConstants;

/**
 * Grabber için gerçek donanım IO implementasyonu.
 * NEO 550 + Through Bore Encoder ile pozisyon kontrolü.
 */
public class GrabberIOReal implements GrabberIO {

    private final SparkMax motor;
    private final SparkMaxConfig config;
    private final SparkAbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController closedLoopController;

    public GrabberIOReal(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
        config = new SparkMaxConfig();

        // Brake mode
        config.idleMode(IdleMode.kBrake);

        // Current limit (NEO 550 için)
        config.smartCurrentLimit(GrabberConstants.kCurrentLimit);

        // PID
        config.closedLoop.pid(
                GrabberConstants.kP,
                GrabberConstants.kI,
                GrabberConstants.kD);

        // Position wrapping
        config.closedLoop.positionWrappingEnabled(false);

        // Absolute encoder (Through Bore)
        config.absoluteEncoder.positionConversionFactor(GrabberConstants.kEncoderPositionFactor);
        config.absoluteEncoder.inverted(false);

        // Output limits
        config.closedLoop.maxOutput(GrabberConstants.kMaxOutput);
        config.closedLoop.minOutput(GrabberConstants.kMinOutput);

        // Apply config
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        absoluteEncoder = motor.getAbsoluteEncoder();
        closedLoopController = motor.getClosedLoopController();
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.positionDegrees = absoluteEncoder.getPosition();
        inputs.velocityDegPerSec = absoluteEncoder.getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setPosition(double positionDegrees) {
        closedLoopController.setReference(positionDegrees, ControlType.kPosition);
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

package frc.robot.subsystems.wrist;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import frc.robot.constants.WristConstants;

/**
 * Wrist IO gerçek implementasyonu.
 * NEO 550 + SparkMax kullanır.
 */
public class WristIOReal implements WristIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController pidController;

    public WristIOReal(int motorID) {
        motor = new SparkMax(motorID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = motor.getClosedLoopController();

        // Konfigürasyon
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(20);
        config.closedLoop.p(WristConstants.kP);
        config.closedLoop.i(WristConstants.kI);
        config.closedLoop.d(WristConstants.kD);
        config.closedLoop.outputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
        motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.positionDegrees = encoder.getPosition() * 360.0; // Rotasyon → Derece
        inputs.velocityDegPerSec = encoder.getVelocity() * 6.0; // RPM → deg/s
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setAngle(double degrees) {
        double rotations = degrees / 360.0;
        pidController.setReference(rotations, ControlType.kPosition);
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

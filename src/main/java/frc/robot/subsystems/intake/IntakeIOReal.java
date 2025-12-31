package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.LimelightHelpers;
import frc.robot.constants.MechanismConstants;

/**
 * Intake mekanizması için gerçek donanım IO implementasyonu.
 * Kraken motor, Limelight 3A (Object Detection), ve MZ80 sensör ile çalışır.
 */
public class IntakeIOReal implements IntakeIO {
    private final TalonFX motor;
    private final DigitalInput mz80Sensor;
    private final String cameraName = "limelight-intake";

    public IntakeIOReal(int motorID) {
        motor = new TalonFX(motorID);
        mz80Sensor = new DigitalInput(MechanismConstants.kIntakeMZ80Port);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // --- Motor Verileri ---
        inputs.velocityRadPerSec = motor.getVelocity().getValueAsDouble() * 2 * Math.PI;
        inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();

        // --- MZ80 Sensör ---
        // MZ80 normalde HIGH, nesne geçince LOW (active-low)
        inputs.intakeSensorTriggered = !mz80Sensor.get();

        // --- Kamera Verileri (Object Detection) ---
        double tv = LimelightHelpers.getTV(cameraName) ? 1.0 : 0.0;

        if (tv > 0.5) {
            inputs.hasGamePiece = true;
            inputs.targetTx = LimelightHelpers.getTX(cameraName);
            inputs.targetTy = LimelightHelpers.getTY(cameraName);
            inputs.targetArea = LimelightHelpers.getTA(cameraName);
            inputs.targetClass = LimelightHelpers.getNeuralClassID(cameraName);
        } else {
            inputs.hasGamePiece = false;
            inputs.targetTx = 0.0;
            inputs.targetTy = 0.0;
            inputs.targetArea = 0.0;
            inputs.targetClass = "";
        }
    }

    @Override
    public void setVoltage(double volts) {
        motor.setControl(new VoltageOut(volts));
    }
}
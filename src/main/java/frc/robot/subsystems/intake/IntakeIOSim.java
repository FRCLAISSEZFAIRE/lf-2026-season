package frc.robot.subsystems.intake;

/**
 * Simülasyonda çalışan Intake implementasyonu.
 * Motor fiziği ve MZ80 sensör simülasyonu.
 */
public class IntakeIOSim implements IntakeIO {

    private double velocityRadPerSec = 0.0;
    private double appliedVolts = 0.0;

    // Sensör simülasyonu
    private int intakeCounter = 0;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Motor simülasyonu
        double accel = (appliedVolts / 12.0) * 100.0 - velocityRadPerSec * 2.0;
        velocityRadPerSec += accel * 0.02;

        inputs.velocityRadPerSec = velocityRadPerSec;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(appliedVolts) * 2.0;

        // Kamera simülasyonu (basit)
        inputs.hasGamePiece = false;
        inputs.targetTx = 0.0;
        inputs.targetTy = 0.0;
        inputs.targetArea = 0.0;
        inputs.targetClass = "";

        // Sensör simülasyonu: Motor çalışırken
        // sanki her 100 döngüde bir nesne geliyormuş gibi simüle et
        inputs.intakeSensorTriggered = false;
        if (appliedVolts > 6.0) {
            intakeCounter++;
            if (intakeCounter % 100 == 0) {
                inputs.intakeSensorTriggered = true;
            }
        } else {
            intakeCounter = 0;
        }
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }
}

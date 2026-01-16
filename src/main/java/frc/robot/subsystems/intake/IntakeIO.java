package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // Motor Verileri
        public double velocityRadPerSec = 0.0;
        public double currentAmps = 0.0;
        public double appliedVolts = 0.0;

        // Pivot Verileri
        public double pivotPositionRad = 0.0;
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;

        // Kamera Verileri (Limelight 3A Object Detection)
        public boolean hasGamePiece = false;
        public double targetTx = 0.0;
        public double targetTy = 0.0;
        public double targetArea = 0.0;
        public String targetClass = "";
        public int targetClassId = -1; // Neural Network class ID

        // MZ80 Sensör Verisi
        public boolean intakeSensorTriggered = false; // Nesne sensörden geçti mi?

        // TOF Sensör Verisi (nesne intake içinde mi?)
        public double tofDistanceMm = 9999.0; // Uzaklık mm cinsinden
    }

    default void updateInputs(IntakeIOInputs inputs) {
    }

    default void setVoltage(double volts) {
    }

    default void setPivotVoltage(double volts) {
    }

    default void setPivotPosition(double rad) {
    }
}
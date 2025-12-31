package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public double anglePositionDegrees = 0.0; // Mevcut açı
        public double angleVelocityDegPerSec = 0.0; // Açısal hız
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(ArmIOInputs inputs) {
    }

    /** Hedef açıya git (derece) */
    default void setAngle(double angleDegrees) {
    }

    /** Voltaj ile manuel kontrol */
    default void setVoltage(double volts) {
    }

    /** Motoru durdur */
    default void stop() {
    }

    /** PID değerlerini güncelle (tuning için) */
    default void setPID(double p, double i, double d) {
    }
}

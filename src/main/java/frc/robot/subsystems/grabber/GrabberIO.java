package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public double positionDegrees = 0.0; // Mevcut açı
        public double velocityDegPerSec = 0.0; // Açısal hız
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(GrabberIOInputs inputs) {
    }

    /** Hedef açıya git (derece) */
    default void setPosition(double positionDegrees) {
    }

    /** Motoru durdur */
    default void stop() {
    }

    /** PID değerlerini güncelle */
    default void setPID(double p, double i, double d) {
    }
}

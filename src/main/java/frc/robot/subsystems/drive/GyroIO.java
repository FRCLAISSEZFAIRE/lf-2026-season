package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        // Yaw (Z ekseni - dönme)
        public double yawPositionRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;

        // Pitch (Y ekseni - öne/arkaya eğme)
        public double pitchDegrees = 0.0;

        // Roll (X ekseni - sağa/sola eğme)
        public double rollDegrees = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {
    }

    default void zeroHeading() {
    }

    /**
     * Simülasyon için Gyro hızını ayarlar.
     * Sadece GyroIOSim tarafından implement edilir.
     */
    default void setYawVelocity(double speedRadPerSec) {
    }
}
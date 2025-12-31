package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs {
        public double positionDegrees = 0.0;
        public double velocityDegPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    default void updateInputs(WristIOInputs inputs) {
    }

    default void setAngle(double degrees) {
    }

    default void setVoltage(double voltage) {
    }

    default void stop() {
    }
}

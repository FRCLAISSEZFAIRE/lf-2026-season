package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        // Sensors
        public boolean fuelPresentBottom = false;
        public boolean fuelPresentLow = false;
        public boolean fuelPresentHigh = false;
        public boolean fuelPresentTop = false;
    }

    default void updateInputs(FeederIOInputs inputs) {
    }

    default void setVoltage(double voltage) {
    }

    default void stop() {
    }
}

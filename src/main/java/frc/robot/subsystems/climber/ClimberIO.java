package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        // Sol motor
        public double leftPositionRotations = 0.0;
        public double leftVelocityRPS = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftTemperatureCelsius = 0.0;

        // Sağ motor
        public double rightPositionRotations = 0.0;
        public double rightVelocityRPS = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightTemperatureCelsius = 0.0;
    }

    default void updateInputs(ClimberIOInputs inputs) {
    }

    /** Her iki motora voltaj uygula */
    default void setVoltage(double volts) {
    }

    /** Motorları durdur */
    default void stop() {
    }
}

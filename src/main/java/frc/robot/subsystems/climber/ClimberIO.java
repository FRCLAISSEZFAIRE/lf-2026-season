package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * Climber IO interface.
 * 2 Kraken motor ile climber sistemi.
 */
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

        // Limit durumları
        public boolean atForwardLimit = false;
        public boolean atReverseLimit = false;
        public boolean isSeated = false; // Tırmanma tamamlandı sensörü
    }

    /** IO girişlerini günceller */
    default void updateInputs(ClimberIOInputs inputs) {
    }

    /** Hedef pozisyon ayarlar (Motion Magic) */
    default void setPosition(double positionRotations) {
    }

    /** Hız ayarlar (manuel kontrol için) */
    default void setVelocity(double velocityRPS) {
    }

    /** Voltaj uygular */
    default void setVoltage(double volts) {
    }

    /** Motorları durdurur */
    default void stop() {
    }

    /** Encoder pozisyonunu sıfırlar */
    default void resetPosition() {
    }
}

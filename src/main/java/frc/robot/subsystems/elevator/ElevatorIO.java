package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public double positionRotations = 0.0; // Rotor pozisyonu
        public double velocityRPS = 0.0; // Rotor hızı (rotasyon/saniye)
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double temperatureCelsius = 0.0;
        public boolean atForwardLimit = false; // Üst limite ulaştı mı?
        public boolean atReverseLimit = false; // Alt limite ulaştı mı?
    }

    default void updateInputs(ElevatorIOInputs inputs) {
    }

    /** Motion Magic ile hedef pozisyona git */
    default void setPosition(double positionRotations) {
    }

    /** Manuel hız kontrolü (Motion Magic Velocity) */
    default void setVelocity(double velocityRPS) {
    }

    /** Voltaj ile manuel kontrol */
    default void setVoltage(double volts) {
    }

    /** Motoru durdur */
    default void stop() {
    }

    /** Encoder pozisyonunu sıfırla */
    default void resetPosition() {
    }
}

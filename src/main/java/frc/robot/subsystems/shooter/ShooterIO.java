package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Flywheel (Fırlatıcı)
        public double flywheelVelocityRadPerSec = 0.0;
        public double flywheelAppliedVolts = 0.0;

        // Turret (Yatay Döner Tabla)
        public double turretAbsolutePositionRad = 0.0;
        public double turretAppliedVolts = 0.0;

        // Hood (Atış Açısı / Dikey)
        public double hoodPositionDegrees = 0.0;
        public double hoodAppliedVolts = 0.0;

        // MZ80 Sensör Verisi
        public boolean shooterSensorTriggered = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {
    }

    // Motor Kontrolleri
    default void setFlywheelVoltage(double volts) {
    }

    default void setTurretVoltage(double volts) {
    }

    // Hood (Atış açısı) kontrolü
    default void setHoodAngle(double angleDegrees) {
    }

    default void setHoodVoltage(double volts) {
    }
}
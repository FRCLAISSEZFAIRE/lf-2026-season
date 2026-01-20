package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        // Flywheel (Fırlatıcı)
        public double flywheelVelocityRadPerSec = 0.0;
        public double flywheelVelocityError = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
        public double flywheelRightVelocityRadPerSec = 0.0;
        public double flywheelRightAppliedVolts = 0.0;

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

    // Flywheel Kontrolleri
    default void setFlywheelVoltage(double volts) {
    }

    default void setFlywheelVelocity(double velocityRPM) {
    }

    default void setFlywheelRightVoltage(double volts) {
    }

    default void setFlywheelRightVelocity(double velocityRPM) {
    }

    // Turret Kontrolleri (Closed-Loop Position)
    /** Turret'i hedef açıya döndürür (radyan) - SparkMax closed-loop */
    default void setTurretPosition(double angleRad) {
    }

    /** Turret voltage control (manuel/test için) */
    default void setTurretVoltage(double volts) {
    }

    // Hood Kontrolleri (Closed-Loop Position)
    /** Hood'u hedef açıya döndürür (derece) - SparkMax closed-loop */
    default void setHoodAngle(double angleDegrees) {
    }

    /** Hood voltage control (manuel/test için) */
    default void setHoodVoltage(double volts) {
    }
}
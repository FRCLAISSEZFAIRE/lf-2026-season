package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;
    }

    /** Inputs nesnesini güncel verilerle doldurur */
    default void updateInputs(ModuleIOInputs inputs) {}

    /** Sürüş motoruna voltaj verir */
    default void setDriveVoltage(double volts) {}

    /** Dönüş motoruna hedef açı verir (PID için) */
    default void setTurnPosition(double angleRad) {}

    /** Sadece test veya simülasyon için voltaj kontrolü */
    default void setTurnVoltage(double volts) {}

    default void setDriveBrakeMode(boolean enable) {}
    default void setTurnBrakeMode(boolean enable) {}
}
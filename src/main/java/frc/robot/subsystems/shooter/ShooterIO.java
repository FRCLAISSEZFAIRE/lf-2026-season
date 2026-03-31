package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double turretPositionDeg = 0.0;
        public double turretVelocityRPM = 0.0;
        public double turretAppliedVolts = 0.0;
        public double[] turretCurrentAmps = new double[] {};
        public boolean turretReverseLimitSwitchPressed = false;

        public double hoodPositionDeg = 0.0;
        public double hoodVelocityRPM = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double[] hoodCurrentAmps = new double[] {};

        public double flywheelVelocityRPM = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double[] flywheelCurrentAmps = new double[] {};
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setTurretPosition(double degrees) {}

    public default void setTurretVoltage(double volts) {}

    public default void resetTurretEncoder() {}

    public default void setHoodPosition(double degrees) {}

    public default void setHoodVoltage(double volts) {}

    public default void resetHoodEncoder(double degrees) {}

    public default void setFlywheelVelocityRPM(double rpm) {}

    public default void stopTurret() {}

    public default void stopHood() {}

    public default void stopFlywheel() {}

    public default void setTurretPID(double p, double i, double d, double maxOutput) {}

    public default void setTurretSoftLimits(double minAngle, double maxAngle) {}

    public default void setHoodPID(double p, double i, double d) {}

    public default void setFlywheelPID(double p, double i, double d, double v) {}

    public default void configTurretGearRatio(double gearRatio) {}

    public default void configHoodGearRatio(double gearRatio) {}

    public default void setFlywheelInverted(boolean inverted) {}
}

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double rollerVelocityRPM = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double[] rollerCurrentAmps = new double[] {};

        public double extensionPositionRevs = 0.0;
        public double extensionVelocityRPM = 0.0;
        public double extensionAppliedVolts = 0.0;
        public double[] extensionCurrentAmps = new double[] {};
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRollerVoltage(double volts) {}

    public default void setRollerVelocity(double rpm) {}

    public default void setExtensionPositionRevs(double revs) {}

    public default void setExtensionVoltage(double volts) {}

    public default void stopRoller() {}

    public default void stopExtension() {}

    public default void resetExtensionEncoder(double revs) {}

    public default void configRoller(double kP, double kI, double kD, double kV, boolean invert) {}

    public default void configExtension(double kP, double kI, double kD, double minRev, double maxRev, boolean softLimitsEnabled, boolean invert) {}
}

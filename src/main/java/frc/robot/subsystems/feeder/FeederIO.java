package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double indexerVelocityRPM = 0.0;
        public double kickerVelocityRPM = 0.0;
        public double indexerAppliedVolts = 0.0;
        public double kickerAppliedVolts = 0.0;
        public double[] indexerCurrentAmps = new double[] {};
        public double[] kickerCurrentAmps = new double[] {};
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    public default void setIndexerVelocity(double rpm) {}

    public default void setKickerVelocity(double rpm) {}

    public default void setIndexerVoltage(double volts) {}

    public default void setKickerVoltage(double volts) {}

    public default void stopIndexer() {}

    public default void stopKicker() {}

    public default void configPID(double indexerP, double indexerI, double indexerD, double indexerFF,
                                  double kickerP, double kickerI, double kickerD, double kickerFF) {}
}

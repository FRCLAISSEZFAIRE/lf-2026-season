package frc.robot.subsystems.feeder;

public class FeederIOSim implements FeederIO {
    private double indexerVelocityRPM = 0.0;
    private double kickerVelocityRPM = 0.0;

    private double indexerTargetRPM = 0.0;
    private double kickerTargetRPM = 0.0;

    private double indexerAppliedVolts = 0.0;
    private double kickerAppliedVolts = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        // Doğrudan hedef takibi — %40 per cycle yakınsama (~0.25s)
        double indexerError = indexerTargetRPM - indexerVelocityRPM;
        indexerVelocityRPM += indexerError * 0.4;

        double kickerError = kickerTargetRPM - kickerVelocityRPM;
        kickerVelocityRPM += kickerError * 0.4;

        // Telemetri voltajları
        indexerAppliedVolts = indexerError * 0.001;
        kickerAppliedVolts = kickerError * 0.001;

        inputs.indexerVelocityRPM = indexerVelocityRPM;
        inputs.indexerAppliedVolts = indexerAppliedVolts;

        inputs.kickerVelocityRPM = kickerVelocityRPM;
        inputs.kickerAppliedVolts = kickerAppliedVolts;
    }

    @Override
    public void setIndexerVelocity(double rpm) {
        indexerTargetRPM = rpm;
    }

    @Override
    public void setKickerVelocity(double rpm) {
        kickerTargetRPM = rpm;
    }

    @Override
    public void setIndexerVoltage(double volts) {
        // Voltaj modunda: yaklaşık RPM dönüşümü
        indexerTargetRPM = volts * 500.0;
    }

    @Override
    public void setKickerVoltage(double volts) {
        kickerTargetRPM = volts * 500.0;
    }

    @Override
    public void stopIndexer() {
        indexerTargetRPM = 0;
    }

    @Override
    public void stopKicker() {
        kickerTargetRPM = 0;
    }
}

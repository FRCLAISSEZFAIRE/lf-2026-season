package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.FeederConstants;
import frc.robot.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    // STATE
    private double indexerTargetRPM = 0;
    private double kickerTargetRPM = 0;

    // TUNABLE PID — INDEXER
    private final TunableNumber indexerKP = new TunableNumber("Feeder", "Indexer kP", FeederConstants.kIndexerP);
    private final TunableNumber indexerKI = new TunableNumber("Feeder", "Indexer kI", FeederConstants.kIndexerI);
    private final TunableNumber indexerKD = new TunableNumber("Feeder", "Indexer kD", FeederConstants.kIndexerD);
    private final TunableNumber indexerKFF = new TunableNumber("Feeder", "Indexer kFF",
            FeederConstants.kIndexerFF);

    // TUNABLE PID — KICKER
    private final TunableNumber kickerKP = new TunableNumber("Feeder", "Kicker kP", FeederConstants.kKickerP);
    private final TunableNumber kickerKI = new TunableNumber("Feeder", "Kicker kI", FeederConstants.kKickerI);
    private final TunableNumber kickerKD = new TunableNumber("Feeder", "Kicker kD", FeederConstants.kKickerD);
    private final TunableNumber kickerKFF = new TunableNumber("Feeder", "Kicker kFF", FeederConstants.kKickerFF);

    // TUNABLE SPEEDS (RPM)
    private final TunableNumber tunableIndexerFeedRPM = new TunableNumber("Feeder", "Indexer Feed RPM",
            FeederConstants.kIndexerFeedRPM);
    private final TunableNumber tunableIndexerSlowRPM = new TunableNumber("Feeder", "Indexer Slow RPM",
            FeederConstants.kIndexerSlowFeedRPM);
    private final TunableNumber tunableIndexerReverseRPM = new TunableNumber("Feeder", "Indexer Reverse RPM",
            FeederConstants.kIndexerReverseRPM);

    private final TunableNumber tunableKickerFeedRPM = new TunableNumber("Feeder", "Kicker Feed RPM",
            FeederConstants.kKickerFeedRPM);
    private final TunableNumber tunableKickerSlowRPM = new TunableNumber("Feeder", "Kicker Slow RPM",
            FeederConstants.kKickerSlowFeedRPM);
    private final TunableNumber tunableKickerReverseRPM = new TunableNumber("Feeder", "Kicker Reverse RPM",
            FeederConstants.kKickerReverseRPM);

    private final TunableNumber rpmTolerance = new TunableNumber("Feeder", "RPM Tolerance",
            FeederConstants.kRPMTolerance);

    // MANUAL OVERRIDE
    private boolean manualOverrideEnabled = false;
    private double manualIndexerRPM = 0;
    private double manualKickerRPM = 0;

    public FeederSubsystem(FeederIO io) {
        this.io = io;
        System.out.println("[Feeder] Başlatıldı — AdvantageKit IO Kullanılıyor");
    }

    private void checkAndApplyTunables() {
        boolean indexerChanged = indexerKP.hasChanged() || indexerKI.hasChanged()
                || indexerKD.hasChanged() || indexerKFF.hasChanged();
        boolean kickerChanged = kickerKP.hasChanged() || kickerKI.hasChanged()
                || kickerKD.hasChanged() || kickerKFF.hasChanged();

        if (indexerChanged || kickerChanged) {
            io.configPID(
                    indexerKP.get(), indexerKI.get(), indexerKD.get(), indexerKFF.get(),
                    kickerKP.get(), kickerKI.get(), kickerKD.get(), kickerKFF.get());

            if (indexerChanged) {
                System.out.println("[Feeder/Indexer] PID Updated: P=" + indexerKP.get() + " FF=" + indexerKFF.get());
            }
            if (kickerChanged) {
                System.out.println("[Feeder/Kicker] PID Updated: P=" + kickerKP.get() + " FF=" + kickerKFF.get());
            }
        }
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        checkAndApplyTunables();

        if (tunableIndexerFeedRPM.hasChanged()) System.out.println("[Feeder] Indexer Feed RPM: " + tunableIndexerFeedRPM.get());
        if (tunableKickerFeedRPM.hasChanged()) System.out.println("[Feeder] Kicker Feed RPM: " + tunableKickerFeedRPM.get());
        if (rpmTolerance.hasChanged()) System.out.println("[Feeder] RPM Tolerance: " + rpmTolerance.get());

        logTelemetry();
    }

    public void setVelocity(double indexerRPM, double kickerRPM) {
        indexerTargetRPM = indexerRPM;
        kickerTargetRPM = kickerRPM;
        io.setIndexerVelocity(indexerRPM);
        io.setKickerVelocity(kickerRPM);
    }

    public void setIndexerVelocity(double rpm) {
        indexerTargetRPM = rpm;
        io.setIndexerVelocity(rpm);
    }

    public void setKickerVelocity(double rpm) {
        kickerTargetRPM = rpm;
        io.setKickerVelocity(rpm);
    }

    public void feed() {
        if (!manualOverrideEnabled) {
            setVelocity(tunableIndexerFeedRPM.get(), tunableKickerFeedRPM.get());
        }
    }

    public void feedSlow() {
        if (!manualOverrideEnabled) {
            setVelocity(tunableIndexerSlowRPM.get(), 0);
        }
    }

    public void reverse() {
        if (!manualOverrideEnabled) {
            setVelocity(tunableIndexerReverseRPM.get(), tunableKickerReverseRPM.get());
        }
    }

    public void stop() {
        if (!manualOverrideEnabled) {
            indexerTargetRPM = 0;
            kickerTargetRPM = 0;
            io.stopIndexer();
            io.stopKicker();
        }
    }

    public void feedIndexer() {
        setIndexerVelocity(tunableIndexerFeedRPM.get());
    }

    public void feedKicker() {
        setKickerVelocity(tunableKickerFeedRPM.get());
    }

    public void stopIndexer() {
        indexerTargetRPM = 0;
        io.stopIndexer();
    }

    public void stopKicker() {
        kickerTargetRPM = 0;
        io.stopKicker();
    }

    public void enableManualOverride() {
        manualOverrideEnabled = true;
        System.out.println("[Feeder] Manual override ENABLED");
    }

    public void disableManualOverride() {
        manualOverrideEnabled = false;
        manualIndexerRPM = 0;
        manualKickerRPM = 0;
        io.stopIndexer();
        io.stopKicker();
        System.out.println("[Feeder] Manual override DISABLED");
    }

    public boolean isManualOverrideEnabled() {
        return manualOverrideEnabled;
    }

    public void setManualIndexerRPM(double rpm) {
        if (manualOverrideEnabled) {
            manualIndexerRPM = rpm;
            io.setIndexerVelocity(rpm);
        }
    }

    public void setManualKickerRPM(double rpm) {
        if (manualOverrideEnabled) {
            manualKickerRPM = rpm;
            io.setKickerVelocity(rpm);
        }
    }

    public void setManualOverrideRPM(double rpm) {
        setManualIndexerRPM(rpm);
        setManualKickerRPM(rpm);
    }

    public void manualFeed() {
        if (manualOverrideEnabled) {
            manualIndexerRPM = tunableIndexerFeedRPM.get();
            manualKickerRPM = tunableKickerFeedRPM.get();
            io.setIndexerVelocity(manualIndexerRPM);
            io.setKickerVelocity(manualKickerRPM);
        }
    }

    public void manualStop() {
        if (manualOverrideEnabled) {
            manualIndexerRPM = 0;
            manualKickerRPM = 0;
        }
        io.stopIndexer();
        io.stopKicker();
    }

    public double getIndexerVelocityRPM() {
        return inputs.indexerVelocityRPM;
    }

    public double getKickerVelocityRPM() {
        return inputs.kickerVelocityRPM;
    }

    public double getVelocityRPM() {
        return getIndexerVelocityRPM();
    }

    public boolean isAtTarget() {
        boolean indexerOk = Math.abs(getIndexerVelocityRPM() - indexerTargetRPM) < rpmTolerance.get();
        boolean kickerOk = Math.abs(getKickerVelocityRPM() - kickerTargetRPM) < rpmTolerance.get();
        return indexerOk && kickerOk;
    }

    public boolean isRunning() {
        return Math.abs(getIndexerVelocityRPM()) > 50 || Math.abs(getKickerVelocityRPM()) > 50;
    }

    private void logTelemetry() {
        Logger.recordOutput("Tuning/Feeder/Indexer/TargetRPM", indexerTargetRPM);
        Logger.recordOutput("Tuning/Feeder/Indexer/ActualRPM", getIndexerVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/Kicker/TargetRPM", kickerTargetRPM);
        Logger.recordOutput("Tuning/Feeder/Kicker/ActualRPM", getKickerVelocityRPM());
        Logger.recordOutput("Tuning/Feeder/ManualOverrideEnabled", manualOverrideEnabled);
        Logger.recordOutput("Tuning/Feeder/ManualIndexerRPM", manualIndexerRPM);
        Logger.recordOutput("Tuning/Feeder/ManualKickerRPM", manualKickerRPM);
    }

    public double getTunableFeedVoltage() {
        return tunableIndexerFeedRPM.get();
    }

    public double getTunableReverseVoltage() {
        return tunableIndexerReverseRPM.get();
    }

    public void setVoltage(double volts) {
        io.setIndexerVoltage(volts);
        io.setKickerVoltage(volts);
    }
}

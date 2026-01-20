package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.constants.ClimberConstants;
import frc.robot.util.TunableNumber;

/**
 * Tırmanma (Climber) alt sistemi.
 * 
 * TunableNumber ile runtime'da ayarlanabilen parametreler:
 * - Preset pozisyonları
 * - Manuel hızlar
 */
public class ClimberSubsystem extends SubsystemBase {

    public enum ClimberPreset {
        HOME, EXTEND, RETRACT, HOLD
    }

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private ClimberPreset currentPreset = ClimberPreset.HOME;
    private Supplier<Double> pitchSupplier = () -> 0.0;
    private Supplier<Double> rollSupplier = () -> 0.0;

    // ===========================================================================
    // TUNABLE PARAMETERS - Kalıcı olarak kaydedilir
    // ===========================================================================
    private final TunableNumber tunableHomePosition;
    private final TunableNumber tunableExtendPosition;
    private final TunableNumber tunableRetractPosition;
    private final TunableNumber tunableHoldPosition;
    private final TunableNumber tunableManualUpVelocity;
    private final TunableNumber tunableManualDownVelocity;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;

        // TunableNumber oluştur
        tunableHomePosition = new TunableNumber("Climber", "Home Position", ClimberConstants.kHomePosition);
        tunableExtendPosition = new TunableNumber("Climber", "Extend Position", ClimberConstants.kClimbExtendPosition);
        tunableRetractPosition = new TunableNumber("Climber", "Retract Position",
                ClimberConstants.kClimbRetractPosition);
        tunableHoldPosition = new TunableNumber("Climber", "Hold Position", ClimberConstants.kClimbHoldPosition);
        tunableManualUpVelocity = new TunableNumber("Climber", "Manual Up Vel", ClimberConstants.kManualUpVelocity);
        tunableManualDownVelocity = new TunableNumber("Climber", "Manual Down Vel",
                ClimberConstants.kManualDownVelocity);
    }

    public void setGyroSuppliers(Supplier<Double> pitchSupplier, Supplier<Double> rollSupplier) {
        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        double avgPosition = getPosition();
        Logger.recordOutput("Climber/AveragePosition", avgPosition);
        Logger.recordOutput("Climber/CurrentPreset", currentPreset.name());
        Logger.recordOutput("Climber/AtTarget", isAtTarget());

        double totalCurrent = inputs.leftCurrentAmps + inputs.rightCurrentAmps;
        Logger.recordOutput("Climber/TotalCurrent", totalCurrent);
        Logger.recordOutput("Climber/LeftStalling", inputs.leftCurrentAmps > ClimberConstants.kStallCurrentThreshold);
        Logger.recordOutput("Climber/RightStalling", inputs.rightCurrentAmps > ClimberConstants.kStallCurrentThreshold);

        double pitch = pitchSupplier.get();
        double roll = rollSupplier.get();
        double maxTilt = Math.max(Math.abs(pitch), Math.abs(roll));
        Logger.recordOutput("Climber/Pitch", pitch);
        Logger.recordOutput("Climber/Roll", roll);
        Logger.recordOutput("Climber/MaxTilt", maxTilt);
        Logger.recordOutput("Climber/TiltWarning", maxTilt > ClimberConstants.kTiltWarningThreshold);
        Logger.recordOutput("Climber/TiltDanger", maxTilt > ClimberConstants.kTiltDangerThreshold);
        Logger.recordOutput("Climber/IsSeated", isSeated());

        // Tunable values log
        Logger.recordOutput("Climber/TunableExtendPos", tunableExtendPosition.get());
        Logger.recordOutput("Climber/TunableRetractPos", tunableRetractPosition.get());
    }

    // ===========================================================================
    // PRESET POSITIONS (Tunable)
    // ===========================================================================

    private double getPresetPosition(ClimberPreset preset) {
        switch (preset) {
            case HOME:
                return tunableHomePosition.get();
            case EXTEND:
                return tunableExtendPosition.get();
            case RETRACT:
                return tunableRetractPosition.get();
            case HOLD:
                return tunableHoldPosition.get();
            default:
                return 0.0;
        }
    }

    private void goToPreset(ClimberPreset preset) {
        currentPreset = preset;
        io.setPosition(getPresetPosition(preset));
    }

    public void extend() {
        goToPreset(ClimberPreset.EXTEND);
    }

    public void retract() {
        goToPreset(ClimberPreset.RETRACT);
    }

    public void hold() {
        goToPreset(ClimberPreset.HOLD);
    }

    public void home() {
        goToPreset(ClimberPreset.HOME);
    }

    // ===========================================================================
    // MANUAL CONTROL (Tunable velocities)
    // ===========================================================================

    public void manualUp() {
        io.setVelocity(tunableManualUpVelocity.get());
    }

    public void manualDown() {
        io.setVelocity(tunableManualDownVelocity.get());
    }

    public void stop() {
        io.stop();
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    // ===========================================================================
    // STATUS
    // ===========================================================================

    public boolean isAtTarget() {
        double avgPosition = getPosition();
        return Math.abs(avgPosition - getPresetPosition(currentPreset)) < ClimberConstants.kPositionTolerance;
    }

    public boolean isSeated() {
        return inputs.isSeated;
    }

    public double getPosition() {
        return (inputs.leftPositionRotations + inputs.rightPositionRotations) / 2.0;
    }

    public boolean isStalling() {
        return inputs.leftCurrentAmps > ClimberConstants.kStallCurrentThreshold ||
                inputs.rightCurrentAmps > ClimberConstants.kStallCurrentThreshold;
    }

    public void resetPosition() {
        io.resetPosition();
        currentPreset = ClimberPreset.HOME;
    }

    // ===========================================================================
    // TUNABLE GETTERS
    // ===========================================================================

    public double getTunableExtendPosition() {
        return tunableExtendPosition.get();
    }

    public double getTunableRetractPosition() {
        return tunableRetractPosition.get();
    }

    public double getTunableHoldPosition() {
        return tunableHoldPosition.get();
    }
}

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.TunableNumber;

/**
 * Besleyici alt sistemi.
 * Intake'ten alınan oyun parçalarını Shooter'a transfer eder.
 * 
 * TunableNumber ile runtime'da ayarlanabilen parametreler.
 */
public class FeederSubsystem extends SubsystemBase {

    private final FeederIO io;
    private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

    // ===========================================================================
    // TUNABLE PARAMETERS - Kalıcı olarak kaydedilir
    // ===========================================================================
    private final TunableNumber tunableFeedVoltage;
    private final TunableNumber tunableReverseVoltage;

    public FeederSubsystem(FeederIO io) {
        this.io = io;

        // TunableNumber oluştur
        tunableFeedVoltage = new TunableNumber("Feeder", "Feed Voltage", 8.0);
        tunableReverseVoltage = new TunableNumber("Feeder", "Reverse Voltage", -6.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);

        // Yakıt Durumu
        String status = "Boş";
        if (inputs.fuelPresentTop) {
            status = "Dolu";
        } else if (inputs.fuelPresentBottom) {
            status = "Yarım";
        }

        Logger.recordOutput("Feeder/Status", status);
        Logger.recordOutput("Feeder/ItemCount", getFuelLevel());
        Logger.recordOutput("Feeder/IsFull", isFuelSystemFull());
        Logger.recordOutput("Feeder/TunableFeedVoltage", tunableFeedVoltage.get());
        Logger.recordOutput("Feeder/TunableReverseVoltage", tunableReverseVoltage.get());

        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Feeder Status", status);
    }

    // ===========================================================================
    // ACTIONS
    // ===========================================================================

    /** İleri besleme (Tunable voltaj ile) */
    public void feed() {
        io.setVoltage(tunableFeedVoltage.get());
    }

    /** Geri çıkarma (Tunable voltaj ile) */
    public void reverse() {
        io.setVoltage(tunableReverseVoltage.get());
    }

    /** Manuel voltaj */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    /** Durdur */
    public void stop() {
        io.stop();
    }

    // ===========================================================================
    // STATUS
    // ===========================================================================

    public boolean isRunning() {
        return Math.abs(inputs.velocityRPM) > 50;
    }

    public double getVelocityRPM() {
        return inputs.velocityRPM;
    }

    // ===========================================================================
    // FUEL TANK LOGIC
    // ===========================================================================

    private boolean isLoading = false;

    public void setLoading(boolean loading) {
        this.isLoading = loading;
    }

    public boolean isLoading() {
        return isLoading;
    }

    public boolean isFuelSystemEmpty() {
        return !inputs.fuelPresentBottom;
    }

    public int getFuelLevel() {
        int level = 0;
        if (inputs.fuelPresentBottom)
            level++;
        if (inputs.fuelPresentTop)
            level++;
        return level;
    }

    public boolean isFuelSystemFull() {
        return inputs.fuelPresentTop;
    }

    // ===========================================================================
    // TUNABLE GETTERS
    // ===========================================================================

    public double getTunableFeedVoltage() {
        return tunableFeedVoltage.get();
    }

    public double getTunableReverseVoltage() {
        return tunableReverseVoltage.get();
    }
}

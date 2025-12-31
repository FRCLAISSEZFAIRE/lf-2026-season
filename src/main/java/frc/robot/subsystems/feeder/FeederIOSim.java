package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Feeder simülasyon implementasyonu.
 */
public class FeederIOSim implements FeederIO {

    private final FlywheelSim sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1), 0.002, 3.0),
            DCMotor.getFalcon500(1));

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        sim.update(0.02);
        inputs.velocityRPM = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVolts = MathUtil.clamp(voltage, -12, 12);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }
}

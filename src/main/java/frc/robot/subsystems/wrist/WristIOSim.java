package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Wrist simülasyon implementasyonu.
 */
public class WristIOSim implements WristIO {

    private final SingleJointedArmSim sim = new SingleJointedArmSim(
            DCMotor.getNeo550(1),
            50.0, // Gear ratio
            0.01, // Moment of inertia (kg*m^2)
            0.15, // Arm length (m)
            Math.toRadians(-45), // Min angle
            Math.toRadians(45), // Max angle
            true, // Simulate gravity
            Math.toRadians(0) // Starting angle
    );

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(WristIOInputs inputs) {
        sim.update(0.02);
        inputs.positionDegrees = Math.toDegrees(sim.getAngleRads());
        inputs.velocityDegPerSec = Math.toDegrees(sim.getVelocityRadPerSec());
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setAngle(double degrees) {
        // Simple P control for simulation
        double error = degrees - Math.toDegrees(sim.getAngleRads());
        setVoltage(MathUtil.clamp(error * 0.1, -6, 6));
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

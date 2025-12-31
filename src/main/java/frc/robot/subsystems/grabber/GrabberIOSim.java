package frc.robot.subsystems.grabber;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.GrabberConstants;

/**
 * Simülasyonda çalışan Grabber implementasyonu.
 * Pozisyon kontrolü simülasyonu.
 */
public class GrabberIOSim implements GrabberIO {

    private double positionDegrees = GrabberConstants.kOpenPosition;
    private double targetPosition = GrabberConstants.kOpenPosition;
    private double velocityDegPerSec = 0.0;

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        // Simple position control simulation
        double error = targetPosition - positionDegrees;
        velocityDegPerSec = MathUtil.clamp(error * 5.0, -45.0, 45.0);

        if (Math.abs(error) < 0.5) {
            velocityDegPerSec = 0;
        }

        positionDegrees += velocityDegPerSec * 0.02;

        inputs.positionDegrees = positionDegrees;
        inputs.velocityDegPerSec = velocityDegPerSec;
        inputs.appliedVolts = velocityDegPerSec / 45.0 * 12.0;
        inputs.currentAmps = Math.abs(inputs.appliedVolts) * 1.5;
    }

    @Override
    public void setPosition(double positionDegrees) {
        targetPosition = positionDegrees;
    }

    @Override
    public void stop() {
        targetPosition = positionDegrees;
    }

    @Override
    public void setPID(double p, double i, double d) {
        // Simülasyonda kullanılmıyor
    }
}

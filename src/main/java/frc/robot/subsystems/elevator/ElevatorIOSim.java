package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ElevatorConstants;

/**
 * Simülasyonda çalışan Elevator implementasyonu.
 * Basit fizik modeli ile Motion Magic davranışı simüle eder.
 */
public class ElevatorIOSim implements ElevatorIO {

    private double positionRotations = 0.0;
    private double velocityRPS = 0.0;
    private double appliedVolts = 0.0;

    private double targetPosition = 0.0;
    private double targetVelocity = 0.0;
    private boolean usePositionControl = true;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (usePositionControl) {
            // Motion Magic position simulation
            double error = targetPosition - positionRotations;
            double maxVelocity = ElevatorConstants.kElevatorMMCV;

            // Profiled movement towards target
            double desiredVelocity = MathUtil.clamp(error * 5.0, -maxVelocity, maxVelocity);
            velocityRPS += (desiredVelocity - velocityRPS) * 0.1; // Smooth acceleration

            // Stop near target
            if (Math.abs(error) < 0.1) {
                velocityRPS = 0;
            }
        } else {
            // Velocity control simulation
            velocityRPS = targetVelocity;
        }

        // Apply soft limits
        if (positionRotations >= ElevatorConstants.kForwardSoftLimit && velocityRPS > 0) {
            velocityRPS = 0;
        }
        if (positionRotations <= ElevatorConstants.kReverseSoftLimit && velocityRPS < 0) {
            velocityRPS = 0;
        }

        // Update position
        positionRotations += velocityRPS * 0.02; // 20ms loop
        positionRotations = MathUtil.clamp(
                positionRotations,
                ElevatorConstants.kReverseSoftLimit,
                ElevatorConstants.kForwardSoftLimit);

        // Approximate voltage
        appliedVolts = velocityRPS / ElevatorConstants.kElevatorMMCV * 12.0;

        // Set outputs
        inputs.positionRotations = positionRotations;
        inputs.velocityRPS = velocityRPS;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(appliedVolts) * 3.0; // Approximate
        inputs.temperatureCelsius = 25.0;
        inputs.atForwardLimit = positionRotations >= ElevatorConstants.kForwardSoftLimit - 0.5;
        inputs.atReverseLimit = positionRotations <= ElevatorConstants.kReverseSoftLimit + 0.5;
    }

    @Override
    public void setPosition(double positionRotations) {
        usePositionControl = true;
        targetPosition = positionRotations;
    }

    @Override
    public void setVelocity(double velocityRPS) {
        usePositionControl = false;
        targetVelocity = velocityRPS;
    }

    @Override
    public void setVoltage(double volts) {
        usePositionControl = false;
        targetVelocity = volts / 12.0 * ElevatorConstants.kElevatorMMCV;
    }

    @Override
    public void stop() {
        usePositionControl = false;
        targetVelocity = 0;
    }

    @Override
    public void resetPosition() {
        positionRotations = 0;
        velocityRPS = 0;
    }
}

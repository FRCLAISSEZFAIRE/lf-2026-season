package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ClimberConstants;

/**
 * Simülasyonda çalışan Climber implementasyonu.
 */
public class ClimberIOSim implements ClimberIO {

    private double leftPosition = 0.0;
    private double rightPosition = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Motor simülasyonu
        double velocity = appliedVolts / 12.0 * 20.0; // Max 20 RPS

        leftPosition += velocity * 0.02;
        rightPosition += velocity * 0.02;

        // Limit kontrolü
        leftPosition = MathUtil.clamp(leftPosition, ClimberConstants.kMinPosition, ClimberConstants.kMaxPosition);
        rightPosition = MathUtil.clamp(rightPosition, ClimberConstants.kMinPosition, ClimberConstants.kMaxPosition);

        // Sol motor
        inputs.leftPositionRotations = leftPosition;
        inputs.leftVelocityRPS = velocity;
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftCurrentAmps = Math.abs(appliedVolts) * 4.0;
        inputs.leftTemperatureCelsius = 25.0 + Math.abs(appliedVolts);

        // Sağ motor
        inputs.rightPositionRotations = rightPosition;
        inputs.rightVelocityRPS = velocity;
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightCurrentAmps = Math.abs(appliedVolts) * 4.0;
        inputs.rightTemperatureCelsius = 25.0 + Math.abs(appliedVolts);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        appliedVolts = 0;
    }
}

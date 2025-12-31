package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ArmConstants;

/**
 * Simülasyonda çalışan Arm implementasyonu.
 */
public class ArmIOSim implements ArmIO {

    private double anglePositionDegrees = ArmConstants.kStowedAngle;
    private double targetAngle = ArmConstants.kStowedAngle;
    private double velocityDegPerSec = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        // Simple simulation towards target
        double error = targetAngle - anglePositionDegrees;
        velocityDegPerSec = MathUtil.clamp(error * 5.0, -60.0, 60.0);

        if (Math.abs(error) < 0.5) {
            velocityDegPerSec = 0;
        }

        anglePositionDegrees += velocityDegPerSec * 0.02;
        anglePositionDegrees = MathUtil.clamp(
                anglePositionDegrees,
                ArmConstants.kAngleLowerLimitDegrees,
                ArmConstants.kAngleUpperLimitDegrees);

        appliedVolts = velocityDegPerSec / 60.0 * 12.0;

        inputs.anglePositionDegrees = anglePositionDegrees;
        inputs.angleVelocityDegPerSec = velocityDegPerSec;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = Math.abs(appliedVolts) * 2.0;
    }

    @Override
    public void setAngle(double angleDegrees) {
        targetAngle = MathUtil.clamp(
                angleDegrees,
                ArmConstants.kAngleLowerLimitDegrees,
                ArmConstants.kAngleUpperLimitDegrees);
    }

    @Override
    public void setVoltage(double volts) {
        targetAngle = anglePositionDegrees + (volts / 12.0 * 30.0);
    }

    @Override
    public void stop() {
        targetAngle = anglePositionDegrees;
    }

    @Override
    public void setPID(double p, double i, double d) {
        // Simülasyonda PID kullanılmıyor
    }
}

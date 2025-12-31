package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ShooterConstants;

/**
 * Simülasyonda çalışan Shooter implementasyonu.
 */
public class ShooterIOSim implements ShooterIO {

    // Flywheel
    private double flywheelVelocityRadPerSec = 0.0;
    private double flywheelAppliedVolts = 0.0;

    // Turret
    private double turretPositionRad = 0.0;
    private double turretVelocityRadPerSec = 0.0;
    private double turretAppliedVolts = 0.0;

    // Hood
    private double hoodPositionDegrees = ShooterConstants.kHoodMidAngle;
    private double hoodTargetDegrees = ShooterConstants.kHoodMidAngle;
    private double hoodAppliedVolts = 0.0;

    // Sensör
    private int shotCounter = 0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Flywheel simülasyonu
        double flywheelAccel = (flywheelAppliedVolts / 12.0) * 50.0 - flywheelVelocityRadPerSec * 0.5;
        flywheelVelocityRadPerSec += flywheelAccel * 0.02;
        flywheelVelocityRadPerSec = Math.max(0, flywheelVelocityRadPerSec);

        // Turret simülasyonu
        double turretAccel = (turretAppliedVolts / 12.0) * 100.0 - turretVelocityRadPerSec * 5.0;
        turretVelocityRadPerSec += turretAccel * 0.02;
        turretPositionRad += turretVelocityRadPerSec * 0.02;
        turretPositionRad = MathUtil.angleModulus(turretPositionRad);

        // Hood simülasyonu
        double hoodError = hoodTargetDegrees - hoodPositionDegrees;
        double hoodVelocity = MathUtil.clamp(hoodError * 3.0, -30.0, 30.0);
        hoodPositionDegrees += hoodVelocity * 0.02;
        hoodPositionDegrees = MathUtil.clamp(
                hoodPositionDegrees,
                ShooterConstants.kHoodMinAngle,
                ShooterConstants.kHoodMaxAngle);
        hoodAppliedVolts = hoodVelocity / 30.0 * 6.0;

        // Inputs güncelle
        inputs.flywheelVelocityRadPerSec = flywheelVelocityRadPerSec;
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;
        inputs.turretAbsolutePositionRad = turretPositionRad;
        inputs.turretAppliedVolts = turretAppliedVolts;
        inputs.hoodPositionDegrees = hoodPositionDegrees;
        inputs.hoodAppliedVolts = hoodAppliedVolts;

        // Sensör simülasyonu
        inputs.shooterSensorTriggered = false;
        if (flywheelVelocityRadPerSec > 100.0) {
            shotCounter++;
            if (shotCounter % 50 == 0) {
                inputs.shooterSensorTriggered = true;
            }
        } else {
            shotCounter = 0;
        }
    }

    @Override
    public void setFlywheelVoltage(double volts) {
        flywheelAppliedVolts = MathUtil.clamp(volts, 0.0, 12.0);
    }

    @Override
    public void setTurretVoltage(double volts) {
        turretAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        hoodTargetDegrees = MathUtil.clamp(
                angleDegrees,
                ShooterConstants.kHoodMinAngle,
                ShooterConstants.kHoodMaxAngle);
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodTargetDegrees = hoodPositionDegrees + (volts / 12.0 * 10.0);
    }
}

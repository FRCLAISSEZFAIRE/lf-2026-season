package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ShooterConstants;

/**
 * Simülasyonda çalışan Shooter implementasyonu.
 * Turret seeding ve wrapping simüle edilir.
 */
public class ShooterIOSim implements ShooterIO {

    // Flywheel
    private double flywheelVelocityRadPerSec = 0.0;
    private double flywheelAppliedVolts = 0.0;
    private double flywheelTargetRPM = 0.0;

    // Turret (Closed-Loop Sim with seeding)
    private double turretPositionDeg = 0.0; // Derece
    private double turretTargetDeg = 0.0;
    private double turretAppliedVolts = 0.0;
    private boolean turretSeeded = false;

    // Hood
    private double hoodPositionDegrees = ShooterConstants.kHoodMidAngle;
    private double hoodTargetDegrees = ShooterConstants.kHoodMidAngle;
    private double hoodAppliedVolts = 0.0;

    private int shotCounter = 0;

    public ShooterIOSim() {
        // Simülasyonda başlangıç pozisyonu rastgele
        seedTurretPosition();
    }

    /**
     * Turret pozisyonunu seed et (simülasyon için rastgele başlangıç)
     */
    private void seedTurretPosition() {
        turretPositionDeg = Math.random() * 60 - 30; // -30 ile 30 arası
        turretSeeded = true;
        System.out.println("[Turret Sim] Seeded position: " + turretPositionDeg + "°");
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // --- FLYWHEEL simülasyonu ---
        double flywheelTargetRadPerSec = flywheelTargetRPM * 2 * Math.PI / 60.0;
        double flywheelError = flywheelTargetRadPerSec - flywheelVelocityRadPerSec;
        flywheelVelocityRadPerSec += flywheelError * 0.1;
        flywheelVelocityRadPerSec = Math.max(0, flywheelVelocityRadPerSec);
        flywheelAppliedVolts = (flywheelVelocityRadPerSec / (7000.0 * 2 * Math.PI / 60.0)) * 12.0;

        // --- TURRET simülasyonu (Continuous Wrapping) ---
        // En kısa yolu hesapla
        double turretError = turretTargetDeg - turretPositionDeg;
        // -180 ile 180 arası normalize et (continuous wrapping)
        while (turretError > 180.0)
            turretError -= 360.0;
        while (turretError < -180.0)
            turretError += 360.0;

        // P kontrolü simüle et
        double turretVelocity = MathUtil.clamp(turretError * ShooterConstants.kTurretDefaultP * 10, -180.0, 180.0);
        turretPositionDeg += turretVelocity * 0.02;

        // Soft limits uygula
        if (ShooterConstants.kTurretSoftLimitsEnabled) {
            turretPositionDeg = MathUtil.clamp(turretPositionDeg,
                    ShooterConstants.kTurretMinAngle,
                    ShooterConstants.kTurretMaxAngle);
        }

        turretAppliedVolts = turretVelocity / 180.0 * 12.0;

        // --- HOOD simülasyonu ---
        double hoodError = hoodTargetDegrees - hoodPositionDegrees;
        double hoodVelocity = MathUtil.clamp(hoodError * ShooterConstants.kHoodP, -30.0, 30.0);
        hoodPositionDegrees += hoodVelocity * 0.02;
        hoodPositionDegrees = MathUtil.clamp(hoodPositionDegrees,
                ShooterConstants.kHoodMinAngle,
                ShooterConstants.kHoodMaxAngle);
        hoodAppliedVolts = hoodVelocity / 30.0 * 6.0;

        // Inputs güncelle
        inputs.flywheelVelocityRadPerSec = flywheelVelocityRadPerSec;
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;
        inputs.turretAbsolutePositionRad = Math.toRadians(turretPositionDeg);
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
        flywheelTargetRPM = (volts / 12.0) * 7000.0;
    }

    @Override
    public void setFlywheelVelocity(double velocityRPM) {
        flywheelTargetRPM = velocityRPM;
    }

    @Override
    public void setTurretPosition(double angleRad) {
        turretTargetDeg = Math.toDegrees(angleRad);
    }

    @Override
    public void setTurretVoltage(double volts) {
        turretTargetDeg = turretPositionDeg + (volts / 12.0 * 30.0);
    }

    @Override
    public void setHoodAngle(double angleDegrees) {
        hoodTargetDegrees = MathUtil.clamp(angleDegrees,
                ShooterConstants.kHoodMinAngle,
                ShooterConstants.kHoodMaxAngle);
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodTargetDegrees = hoodPositionDegrees + (volts / 12.0 * 10.0);
    }
}

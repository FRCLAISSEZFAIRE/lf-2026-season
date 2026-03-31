package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private double flywheelVelocityRPM = 0.0;
    private double turretPositionDeg = 0.0;
    private double hoodPositionDeg = ShooterConstants.kHoodHomeAngle;

    private double flywheelTargetRPM = 0.0;
    private double turretTargetDeg = 0.0;
    private double hoodTargetDeg = ShooterConstants.kHoodHomeAngle;

    private double flywheelAppliedVolts = 0.0;
    private double turretAppliedVolts = 0.0;
    private double hoodAppliedVolts = 0.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // --- Flywheel: Kraken X60 gerçekçi hız (simülasyon) ---
        double flywheelError = flywheelTargetRPM - flywheelVelocityRPM;
        flywheelVelocityRPM += flywheelError * 0.35; // %35 per cycle → ~0.3s'de hedefe ulaşır

        // --- Turret: NEO hızlı pozisyon takibi (simülasyon) ---
        double turretError = turretTargetDeg - turretPositionDeg;
        turretPositionDeg += turretError * 0.4; // %40 per cycle → ~0.25s'de hedefe ulaşır
        turretPositionDeg = MathUtil.clamp(turretPositionDeg,
                ShooterConstants.kTurretMinAngle, ShooterConstants.kTurretMaxAngle);

        // --- Hood: NEO 550 hızlı pozisyon takibi (simülasyon) ---
        double hoodError = hoodTargetDeg - hoodPositionDeg;
        hoodPositionDeg += hoodError * 0.5; // %50 per cycle → ~0.2s'de hedefe ulaşır
        hoodPositionDeg = MathUtil.clamp(hoodPositionDeg,
                ShooterConstants.kHoodMinAngle, ShooterConstants.kHoodMaxAngle);

        // Voltaj hesaplaması (sadece telemetri için)
        turretAppliedVolts = turretError * 0.1;
        hoodAppliedVolts = hoodError * 0.1;
        flywheelAppliedVolts = flywheelError * 0.001;

        inputs.flywheelVelocityRPM = flywheelVelocityRPM;
        inputs.flywheelAppliedVolts = flywheelAppliedVolts;

        inputs.turretPositionDeg = turretPositionDeg;
        inputs.turretAppliedVolts = turretAppliedVolts;

        inputs.hoodPositionDeg = hoodPositionDeg;
        inputs.hoodAppliedVolts = hoodAppliedVolts;
    }

    @Override
    public void setFlywheelVelocityRPM(double rpm) {
        flywheelTargetRPM = rpm;
    }

    @Override
    public void setTurretPosition(double degrees) {
        turretTargetDeg = degrees;
    }

    @Override
    public void setHoodPosition(double degrees) {
        hoodTargetDeg = degrees;
    }

    @Override
    public void setTurretVoltage(double volts) {
        // Voltaj modunda: doğrudan pozisyonu kaydır
        turretTargetDeg = turretPositionDeg + volts * 5.0;
    }

    @Override
    public void setHoodVoltage(double volts) {
        hoodTargetDeg = hoodPositionDeg + volts * 2.0;
    }

    @Override
    public void stopFlywheel() {
        flywheelTargetRPM = 0.0;
    }

    @Override
    public void stopTurret() {
        turretTargetDeg = turretPositionDeg; // Olduğu yerde dur
    }

    @Override
    public void stopHood() {
        hoodTargetDeg = hoodPositionDeg; // Olduğu yerde dur
    }

    @Override
    public void resetTurretEncoder() {
        turretPositionDeg = 0.0;
        turretTargetDeg = 0.0;
    }

    @Override
    public void resetHoodEncoder(double degrees) {
        hoodPositionDeg = degrees;
        hoodTargetDeg = degrees;
    }
}

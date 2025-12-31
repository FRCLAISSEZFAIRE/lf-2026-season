package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Simülasyonda çalışan Swerve Module implementasyonu.
 * Basit fizik modeli ile sürüş ve dönüş simülasyonu.
 */
public class ModuleIOSim implements ModuleIO {

    // Simülasyon durumu
    private double drivePositionRad = 0.0;
    private double driveVelocityRadPerSec = 0.0;
    private double driveAppliedVolts = 0.0;

    private double turnAbsolutePositionRad = 0.0;
    private double turnVelocityRadPerSec = 0.0;
    private double turnAppliedVolts = 0.0;

    // Motor modelleri
    private static final DCMotor driveMotor = DCMotor.getNeoVortex(1);
    private static final DCMotor turnMotor = DCMotor.getNEO(1);

    // Simülasyon sabitleri
    private static final double kDriveGearRatio = 5.08; // L3 ratio
    private static final double kTurnGearRatio = 21.4; // MaxSwerve turn ratio

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Drive simülasyonu - basit first-order model
        // Hedef hız = (voltage / 12) * maxVelocity
        // Kraken X60 Free Speed: ~6000 RPM (~628 rad/s)
        double maxDriveVelocity = frc.robot.constants.ModuleConstants.kDrivingMotorFreeSpeedRps * 2 * Math.PI; // rad/s (motor shaft)
        double targetDriveVelocity = (driveAppliedVolts / 12.0) * maxDriveVelocity;
        
        // Smooth towards target (first-order lag)
        driveVelocityRadPerSec += (targetDriveVelocity - driveVelocityRadPerSec) * 0.1;
        drivePositionRad += driveVelocityRadPerSec * 0.02;

        // Turn simülasyonu - daha hızlı yanıt
        double maxTurnVelocity = 50.0; // rad/s
        double targetTurnVelocity = (turnAppliedVolts / 12.0) * maxTurnVelocity;
        turnVelocityRadPerSec += (targetTurnVelocity - turnVelocityRadPerSec) * 0.2;
        turnAbsolutePositionRad += turnVelocityRadPerSec * 0.02;

        // Açıyı 0-2π arasında tut
        turnAbsolutePositionRad = MathUtil.angleModulus(turnAbsolutePositionRad);
        if (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2 * Math.PI;
        }

        // Inputs'ı güncelle
        inputs.drivePositionRad = drivePositionRad;
        inputs.driveVelocityRadPerSec = driveVelocityRadPerSec;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveAppliedVolts) * 2.0; // Yaklaşık

        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnVelocityRadPerSec = turnVelocityRadPerSec;
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnAppliedVolts) * 1.0; // Yaklaşık
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void setTurnPosition(double angleRad) {
        // Basit P kontrolü ile hedef açıya git
        double error = MathUtil.angleModulus(angleRad - turnAbsolutePositionRad);
        turnAppliedVolts = MathUtil.clamp(error * 8.0, -12.0, 12.0);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}

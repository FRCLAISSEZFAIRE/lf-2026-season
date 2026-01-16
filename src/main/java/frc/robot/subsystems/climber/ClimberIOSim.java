package frc.robot.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.ClimberConstants;

/**
 * Climber için simülasyon IO implementasyonu.
 * WPILib ElevatorSim kullanır (Dikey hareket).
 */
public class ClimberIOSim implements ClimberIO {

    // Elevator simülasyonu (her iki motor için birleşik)
    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(2), // 2 Kraken motor
            10.0, // Dişli oranı
            5.0, // Taşınan kütle (kg)
            0.02, // Drum yarıçapı (m)
            0.0, // Min yükseklik (m)
            1.5, // Max yükseklik (m)
            true, // Yer çekimi simüle et
            0.0 // Başlangıç pozisyonu
    );

    // Hedef pozisyon (rotor rotasyonları)
    private double targetPosition = 0.0;
    private double appliedVolts = 0.0;

    // Rotasyonu metreye çevirme faktörü
    private static final double ROTATIONS_PER_METER = 66.67; // ~100 rotasyon = 1.5m

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        // Simülasyonu güncelle (20ms)
        elevatorSim.update(0.02);

        double positionMeters = elevatorSim.getPositionMeters();
        double velocityMps = elevatorSim.getVelocityMetersPerSecond();
        double positionRotations = positionMeters * ROTATIONS_PER_METER;
        double velocityRPS = velocityMps * ROTATIONS_PER_METER;

        // Sol motor (simüle)
        inputs.leftPositionRotations = positionRotations;
        inputs.leftVelocityRPS = velocityRPS;
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftCurrentAmps = elevatorSim.getCurrentDrawAmps() / 2.0;
        inputs.leftTemperatureCelsius = 25.0;

        // Sağ motor (aynı değerler)
        inputs.rightPositionRotations = positionRotations;
        inputs.rightVelocityRPS = velocityRPS;
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightCurrentAmps = elevatorSim.getCurrentDrawAmps() / 2.0;
        inputs.rightTemperatureCelsius = 25.0;

        // Limit durumları
        inputs.atForwardLimit = positionRotations >= ClimberConstants.kForwardSoftLimit - 0.5;
        inputs.atReverseLimit = positionRotations <= ClimberConstants.kReverseSoftLimit + 0.5;

        // Simülasyonda Retract seviyesine geldiğinde "oturdu" varsayalım
        inputs.isSeated = positionRotations <= ClimberConstants.kClimbRetractPosition + 2.0;
    }

    @Override
    public void setPosition(double positionRotations) {
        targetPosition = positionRotations;
        // Basit P kontrolü ile pozisyona git
        double currentPosition = elevatorSim.getPositionMeters() * ROTATIONS_PER_METER;
        double error = positionRotations - currentPosition;
        double voltage = Math.max(-12, Math.min(12, error * 0.5)); // Basit P
        setVoltage(voltage);
    }

    @Override
    public void setVelocity(double velocityRPS) {
        // Hızı voltaja çevir (basit model)
        double voltage = velocityRPS / 10.0 * 12.0;
        setVoltage(voltage);
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = Math.max(-12, Math.min(12, volts));
        elevatorSim.setInputVoltage(appliedVolts);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

    @Override
    public void resetPosition() {
        // Simülasyonda pozisyon resetleme mümkün değil, sadece hedefi sıfırla
        targetPosition = 0.0;
    }
}

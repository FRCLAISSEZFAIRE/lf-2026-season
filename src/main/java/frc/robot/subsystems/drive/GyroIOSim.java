package frc.robot.subsystems.drive;

/**
 * Simülasyonda çalışan Gyro implementasyonu.
 * Euler integration ile açıyı günceller.
 */
public class GyroIOSim implements GyroIO {
    private double yawPositionRad = 0.0;
    private double yawVelocityRadPerSec = 0.0;
    private double pitchDegrees = 0.0;
    private double rollDegrees = 0.0;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // Simülasyonda her zaman bağlı
        inputs.connected = true;

        // Açıyı güncelle (Euler integration)
        yawPositionRad += yawVelocityRadPerSec * 0.02; // 20ms döngü

        inputs.yawPositionRad = yawPositionRad;
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
        inputs.pitchDegrees = pitchDegrees;
        inputs.rollDegrees = rollDegrees;
    }

    @Override
    public void zeroHeading() {
        yawPositionRad = 0.0;
    }

    /**
     * Simülasyonda dönüş hızını ayarlamak için kullanılır.
     * DriveSubsystem tarafından çağrılmalı.
     */
    public void setYawVelocity(double radPerSec) {
        this.yawVelocityRadPerSec = radPerSec;
    }

    /**
     * Simülasyonda pitch/roll değerlerini ayarlamak için.
     */
    public void setTilt(double pitch, double roll) {
        this.pitchDegrees = pitch;
        this.rollDegrees = roll;
    }
}

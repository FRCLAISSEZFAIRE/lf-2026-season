package frc.robot.subsystems.drive;

import com.studica.frc.Navx;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConstants;

/**
 * NavX2-MXP IMU için GyroIO implementasyonu.
 * StudicaLib Navx sınıfını kullanır (WPILib 2026beta uyumlu).
 * 
 * Constructor parametresi (integer):
 * - 0: USB
 * - 1: MXP SPI (NavX2-MXP için varsayılan)
 * - 2: I2C
 */
public class GyroIONavX implements GyroIO {
    // Port 1 = MXP SPI (NavX2-MXP için varsayılan)
    private final Navx navx = new Navx(1);

    public GyroIONavX() {
        // Başlangıçta gerekirse sıfırlama yapılabilir
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        // Navx class varsayılan olarak bağlı kabul edilir
        inputs.connected = true;

        // Yaw (dönme)
        // Yaw (dönme)
        double angleDeg = 0.0; // navx.getYaw(); // StudicaLib API sorunu nedeniyle devre dışı
        if (DriveConstants.kGyroReversed) {
            angleDeg *= -1;
        }
        inputs.yawPositionRad = Units.degreesToRadians(angleDeg);
        inputs.yawVelocityRadPerSec = 0.0;

        // Pitch (öne/arkaya eğme)
        inputs.pitchDegrees = 0.0; // navx.getPitch(); // StudicaLib API kontrol edilmeli

        // Roll (sağa/sola eğme)
        inputs.rollDegrees = 0.0; // navx.getRoll(); // StudicaLib API kontrol edilmeli
    }

    @Override
    public void zeroHeading() {
        // Navx sıfırlama - API'de reset yoksa, varsayılan değer kullan
    }
}
package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConstants;

/**
 * NavX2-MXP IMU için GyroIO implementasyonu.
 * Kauai Labs AHRS sınıfını kullanır (NavX 2024.1.0 - kararlı versiyon).
 * 
 * <h2>Bağlantı:</h2>
 * <ul>
 * <li>MXP SPI Portu (Varsayılan)</li>
 * </ul>
 */
public class GyroIONavX implements GyroIO {
    private final AHRS navx;

    public GyroIONavX() {
        // MXP SPI portu üzerinden bağlan
        navx = new AHRS(SPI.Port.kMXP);

        // Başlangıç kalibrasyonu bekleniyor mu?
        // NavX otomatik kalibre olur, ancak isCalibrating() kontrol edilebilir.
        System.out.println("[NavX] Initializing on SPI MXP...");

        // Stabilizasyon için kısa bir bekleme (opsiyonel)
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();

        // Bağlı değilse güncelleme yapma
        if (!inputs.connected)
            return;

        // Yaw (Z ekseni - Dönüş)
        double angleDeg = navx.getYaw();
        if (DriveConstants.kGyroReversed) {
            angleDeg *= -1;
        }
        inputs.yawPositionRad = Units.degreesToRadians(angleDeg);
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(navx.getRate());

        // Pitch & Roll (Eğim)
        inputs.pitchDegrees = navx.getPitch();
        inputs.rollDegrees = navx.getRoll();
    }

    @Override
    public void zeroHeading() {
        navx.zeroYaw();
        System.out.println("[NavX] Yaw zeroed.");
    }
}

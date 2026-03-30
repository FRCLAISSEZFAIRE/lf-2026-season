package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Utility class to calculate virtual targets for shooting on the move.
 */
public final class MovingShootUtil {

    private MovingShootUtil() {}

    /**
     * Calculates the virtual target position required for Shoot-on-the-Move.
     * 
     * @param realTarget      Orijinal hedefin sahadaki koordinatları (HubLocation).
     * @param turretFieldPos  Turret'in sahadaki anlık pozisyonu.
     * @param robotSpeeds     Robotun sahadaki anlık hız vektörü (Field-Relative).
     * @param shotSpeedMps    Topun fırlatılma anındaki ortalama çıkış hızı (m/s).
     * @param originalDist    Taret başlangıç mesafesi (Turret ile RealTarget arası) (m).
     * @return                Nişan alınması gereken sanal hedefin (Virtual Target) koordinatları.
     */
    public static Translation2d getVirtualTarget(
            Translation2d realTarget, 
            Translation2d turretFieldPos, 
            ChassisSpeeds robotSpeeds, 
            double shotSpeedMps,
            double originalDist) {
        
        // Atış hızı 0 veya çok küçükse (0'a bölme hatasını önlemek için)
        if (shotSpeedMps < 0.1) {
            return realTarget;
        }

        // 1. Time of Flight (ToF) hesaplaması: t = mesafe / atış_hızı
        // İki iterasyon da yapılabilir ama FRC için çoğu zaman baz mesafe üzerinden hesaplanan uçuş süresi yeterlidir.
        double timeOfFlight = originalDist / shotSpeedMps;

        // 2. Robotun ToF süresince topla beraber kat edeceği vektörel uzaklık kayması
        // Top atıldığı an robotun hız vektörünü "ebediyen" (havada kaldığı süre boyunca) kopyalar.
        double driftX = robotSpeeds.vxMetersPerSecond * timeOfFlight;
        double driftY = robotSpeeds.vyMetersPerSecond * timeOfFlight;

        // 3. Sanal Hedef: Gerçek hedefi, robotun hareket vektörünün TERSİNE, uçuş süresindeki drift kadar kaydırıyoruz.
        // Formül: V_T = T - V_R * t
        double virtualX = realTarget.getX() - driftX;
        double virtualY = realTarget.getY() - driftY;

        return new Translation2d(virtualX, virtualY);
    }
}

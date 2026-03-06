package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Çift Limelight pose tahmini ve sürücü kamerası için vision sabitleri.
 * 
 * <h2>Kamera Yapılandırması:</h2>
 * <ul>
 * <li><b>limelight-left:</b> Sol arka köşe, 45° dışa dönük — Pose Tahmini
 * (MegaTag 2)</li>
 * <li><b>limelight-right:</b> Sağ arka köşe, 45° dışa dönük — Pose Tahmini
 * (MegaTag 2)</li>
 * <li><b>driver-cam:</b> Intake yakınında — Sürücü Kamerası (Sadece Video
 * Akışı)</li>
 * </ul>
 */
public final class VisionConstants {

        // ==================== KAMERA İSİMLERİ ====================
        /** Sol Limelight 3A — sol arka köşe, pose tahmini */
        public static final String kLimelightLeft = "limelight-left";

        /** Sağ Limelight 3 — sağ arka köşe, pose tahmini */
        public static final String kLimelightRight = "limelight-right";

        /** Sürücü kamerası — sadece video akışı (algılama yok) */
        public static final String kDriverCamera = "driver-cam";

        // ==================== KAMERA DÖNÜŞÜMLERİ ====================
        // Robot koordinat sistemi: X = ileri, Y = sol, Z = yukarı
        // Dönüşümler: Robot Merkezi → Kamera

        // Fiziksel ölçümler (robotunuza göre ayarlayın!)
        private static final double kCameraXBackOffset = Units.inchesToMeters(-12.0); // Merkezin 12" arkasında
        private static final double kCameraYSideOffset = Units.inchesToMeters(10.0); // Merkezden 10" yanda
        private static final double kCameraZHeight = Units.inchesToMeters(8.0); // Yerden 8" yükseklikte
        private static final double kCameraPitchDegrees = -15.0; // 15° aşağı eğik

        /**
         * Sol kamera dönüşümü: sol arka köşe, 45° dışa dönük (arka-sola bakıyor).
         * Yaw = -135° → kamera arka-sol çaprazına bakar.
         */
        public static final Transform3d kLeftCameraToRobot = new Transform3d(
                        new Translation3d(
                                        kCameraXBackOffset, // X: robot merkezinin arkasında
                                        kCameraYSideOffset, // Y: merkezin solunda
                                        kCameraZHeight // Z: yükseklik
                        ),
                        new Rotation3d(
                                        0, // Roll: 0
                                        Math.toRadians(kCameraPitchDegrees), // Pitch: aşağı eğik
                                        Math.toRadians(-135) // Yaw: arka-sola bakıyor (45° dışa)
                        ));

        /**
         * Sağ kamera dönüşümü: sağ arka köşe, 45° dışa dönük (arka-sağa bakıyor).
         * Yaw = 135° → kamera arka-sağ çaprazına bakar.
         */
        public static final Transform3d kRightCameraToRobot = new Transform3d(
                        new Translation3d(
                                        kCameraXBackOffset, // X: robot merkezinin arkasında
                                        -kCameraYSideOffset, // Y: merkezin sağında (negatif)
                                        kCameraZHeight // Z: yükseklik
                        ),
                        new Rotation3d(
                                        0, // Roll: 0
                                        Math.toRadians(kCameraPitchDegrees), // Pitch: aşağı eğik
                                        Math.toRadians(135) // Yaw: arka-sağa bakıyor (45° dışa)
                        ));

        // ==================== PIPELINE'LAR ====================
        /** AprilTag algılama pipeline indeksi */
        public static final int kAprilTagPipelineIndex = 0;

        // ==================== FİLTRELEME EŞİKLERİ ====================
        /** Vision verisi reddedilmeden önceki maksimum gyro hızı (derece/saniye) */
        public static final double kMaxGyroRateForVision = 720.0;

        /** Belirsizlik artırılmadan önceki maksimum tag mesafesi (metre) */
        public static final double kMaxReliableTagDistance = 4.0;

        /** Yüksek belirsizlik için minimum tag mesafesi eşiği */
        public static final double kHighUncertaintyTagDistance = 3.0;
}
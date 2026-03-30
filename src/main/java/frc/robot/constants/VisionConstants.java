package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Çift Limelight pose tahmini ve sürücü kamerası için vision sabitleri.
 * 
 * <h2>Kamera Yapılandırması:</h2>
 * <ul>
 * <li><b>limelight-left:</b> Sol arka köşe, 30° dışa dönük — Pose Tahmini
 * (MegaTag 2)</li>
 * <li><b>limelight-right:</b> Sağ arka köşe, 30° dışa dönük — Pose Tahmini
 * (MegaTag 2)</li>
 * <li><b>driver-cam:</b> Intake yakınında — Sürücü Kamerası (Sadece Video
 * Akışı)</li>
 * </ul>
 */
public final class VisionConstants {

        // ==================== KAMERA İSİMLERİ ====================
        /** Sol Limelight 4 — sol arka köşe, pose tahmini */
        public static final String kLimelightLeft = "limelight-left";

        /** Sağ Limelight 3 — sağ arka köşe, pose tahmini */
        public static final String kLimelightRight = "limelight-right";

        /** Sürücü kamerası — sadece video akışı (algılama yok) */
        public static final String kDriverCamera = "driver-cam";

        // ==================== KAMERA DÖNÜŞÜMLERİ ====================
        // Robot koordinat sistemi: X = ileri, Y = sol, Z = yukarı
        // Dönüşümler: Robot Merkezi → Kamera

        // Fiziksel ölçümler (robotunuza göre ayarlayın!)
        private static final double kCameraXBackOffset = -0.29; // Merkezin 30cm arkasında (metre)
        private static final double kCameraYSideOffset = -0.32; // Merkezden 32cm yanda (metre)
        private static final double kCameraZHeight = 0.25; // Yerden 26.5cm yükseklikte (metre)
        private static final double kCameraPitchDegrees = 15.0; // 15° yukarı eğik

        /**
         * Sol kamera dönüşümü: sol arka köşe, 30° dışa dönük (arka-sola bakıyor).
         * Yaw = -150° → kamera arka-sol çaprazına bakar.
         */
        public static final Transform3d kLeftCameraToRobot = new Transform3d(
                        new Translation3d(
                                        kCameraXBackOffset - 0.105, // X: robot merkezinin arkasında
                                        kCameraYSideOffset + 0.1, // Y: merkezin solunda
                                        kCameraZHeight // Z: yükseklik
                        ),
                        new Rotation3d(
                                        0, // Roll: 0
                                        Math.toRadians(kCameraPitchDegrees), // Pitch: yukarı eğik
                                        Math.toRadians(150) // Yaw: arka-sola bakıyor (30° dışa)
                        ));

        /**
         * Sağ kamera dönüşümü: sağ arka köşe, 30° dışa dönük (arka-sağa bakıyor).
         * Yaw = 150° → kamera arka-sağ çaprazına bakar.
         */
        public static final Transform3d kRightCameraToRobot = new Transform3d(
                        new Translation3d(
                                        kCameraXBackOffset, // X: robot merkezinin arkasında
                                        -kCameraYSideOffset, // Y: merkezin sağında (negatif)
                                        kCameraZHeight + 0.015 // Z: yükseklik
                        ),
                        new Rotation3d(
                                        0, // Roll: 0
                                        Math.toRadians(kCameraPitchDegrees), // Pitch: yukarı eğik
                                        Math.toRadians(-150) // Yaw: arka-sağa bakıyor (30° dışa)
                        ));

        // ==================== PIPELINE'LAR ====================
        /** AprilTag algılama pipeline indeksi */
        public static final int kAprilTagPipelineIndex = 0;

        // ==================== FİLTRELEME EŞİKLERİ ====================
        /** Vision verisi reddedilmeden önceki maksimum gyro hızı (derece/saniye) */
        public static final double kMaxGyroRateForVision = 360.0;

        /** Belirsizlik artırılmadan önceki maksimum tag mesafesi (metre) */
        public static final double kMaxReliableTagDistance = 4.0;

        /** Yüksek belirsizlik için minimum tag mesafesi eşiği */
        public static final double kHighUncertaintyTagDistance = 3.0;
}
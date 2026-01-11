package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Saha üzerindeki sabit pozisyonlar ve önemli noktalar.
 * FRC 2026 REBUILT sahası: 16.49m x 8.10m (iç oyun alanı)
 * Dış ölçüler: 19.35m x 8.48m
 */
public final class FieldConstants {

    // === SAHA ÖLÇÜLERİ (FRC 2026 REBUILT) ===
    /** İç oyun alanı uzunluğu (54 ft 1 in) */
    public static final double kFieldLengthMeters = 16.49;
    /** İç oyun alanı genişliği (26 ft 7 in) */
    public static final double kFieldWidthMeters = 8.10;
    /** Dış saha uzunluğu (63 ft 6 in) */
    public static final double kFieldOuterLengthMeters = 19.35;
    /** Dış saha genişliği (27 ft 10 in) */
    public static final double kFieldOuterWidthMeters = 8.48;

    // === HUB POZİSYONLARI (REBUILT 2026) ===
    // Hub'lar sahanın iki ucunda, ittifak zone'larında konumlu
    // Hub üst açıklığı: ~72 inç (1.83m) yükseklikte
    public static final double kHubHeightMeters = 1.83;
    public static final Pose2d kBlueHubPose = new Pose2d(1.0, 4.05, Rotation2d.fromDegrees(0));
    public static final Pose2d kRedHubPose = new Pose2d(15.49, 4.05, Rotation2d.fromDegrees(180));

    // === DEPOT POZİSYONLARI (Fuel alma noktaları) ===
    public static final Pose2d kBlueDepotPose = new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0));
    public static final Pose2d kRedDepotPose = new Pose2d(14.99, 1.0, Rotation2d.fromDegrees(180));

    // === OUTPOST POZİSYONLARI (Human Player Fuel verme noktaları) ===
    public static final Pose2d kBlueOutpostPose = new Pose2d(15.0, 7.0, Rotation2d.fromDegrees(180));
    public static final Pose2d kRedOutpostPose = new Pose2d(1.49, 7.0, Rotation2d.fromDegrees(0));

    // === ATIŞ PARAMETRELERİ ===
    /** İdeal atış mesafesi (Hub'dan uzaklık) */
    public static final double kIdealShootingDistanceMeters = 3.0;

    // === ALLIANCE ZONE SINIRLARI ===
    /** Blue Alliance Zone X sınırı (sol taraf) */
    public static final double kBlueAllianceZoneMaxX = 4.0;
    /** Red Alliance Zone X sınırı (sağ taraf) */
    public static final double kRedAllianceZoneMinX = 12.49;

    // === TOWER POZİSYONLARI (Tırmanma yapısı) ===
    // Tower saha ortasında, her iki ittifak tarafından erişilebilir
    public static final Translation2d kTowerCenter = new Translation2d(8.245, 4.05);
    public static final double kTowerClimbHeightMeters = 1.5; // Yaklaşık tırmanma yüksekliği

    // === HUB SCORING POSITIONS (FRC 2026 REBUILT) ===
    // 6 Adet skor noktası. POV tuşları ile gezilebilir.
    // Blue Alliance tarafındaki Hub etrafındaki yaklaşım noktaları
    public static final Pose2d[] kHubScoringPoses = new Pose2d[] {
            new Pose2d(3.5, 4.05, Rotation2d.fromDegrees(0)), // Pozisyon 1 (Ön - Hub'a bakan)
            new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(-30)), // Pozisyon 2 (Sol Ön)
            new Pose2d(3.0, 3.1, Rotation2d.fromDegrees(30)), // Pozisyon 3 (Sağ Ön)
            new Pose2d(4.0, 5.5, Rotation2d.fromDegrees(-45)), // Pozisyon 4 (Sol Uzak)
            new Pose2d(4.0, 2.6, Rotation2d.fromDegrees(45)), // Pozisyon 5 (Sağ Uzak)
            new Pose2d(4.5, 4.05, Rotation2d.fromDegrees(0)) // Pozisyon 6 (Merkez Uzak)
    };

    // === Legacy: Reef Scoring Poses (Uyumluluk için) ===
    @Deprecated
    public static final Pose2d[] kReefScoringPoses = kHubScoringPoses;

    // === SOURCE/DEPOT POSITIONS ===
    // 2 Adet Depot noktası. POV Sağ/Sol ile gezilebilir.
    public static final Pose2d[] kSourcePoses = new Pose2d[] {
            kBlueDepotPose,
            kBlueOutpostPose
    };

    // === FUEL ÖZELLİKLERİ ===
    /** Fuel çapı (5.91 inç) */
    public static final double kFuelDiameterMeters = 0.15; // 5.91 inch = ~15cm

    private FieldConstants() {
        // Utility class - instantiation prevented
    }
}

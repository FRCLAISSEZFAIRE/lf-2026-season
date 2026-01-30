package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Saha üzerindeki sabit pozisyonlar ve önemli noktalar.
 * FRC 2026 REBUILT sahası: 16.49m x 8.10m (iç oyun alanı)
 */
public final class FieldConstants {

        // === SAHA ÖLÇÜLERİ (FRC 2026 REBUILT) ===
        public static final double kFieldLengthMeters = 16.49;
        public static final double kFieldWidthMeters = 8.10;

        // === HUB POZİSYONLARI (REBUILT 2026) ===
        // Her ittifakın kendi Hub'ı var
        public static final double kHubHeightMeters = 1.83; // Hub üst açıklığı yüksekliği
        public static final Pose2d kBlueHubPose = new Pose2d(1.0, 4.05, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedHubPose = new Pose2d(15.49, 4.05, Rotation2d.fromDegrees(180));

        // Speaker pozisyonları (Shooter auto-aim hedefleri)
        public static final Translation2d kBlueSpeakerPosition = new Translation2d(1.0, 4.05);
        public static final Translation2d kRedSpeakerPosition = new Translation2d(15.49, 4.05);

        // === BAŞLANGIÇ POZİSYONLARI ===
        // Robot başlangıçta Hub'a doğru bakar
        // Blue: Sahanın sol tarafında, Hub'a (sağa) bakar (0°)
        // Red: Sahanın sağ tarafında, Hub'a (sola) bakar (180°)
        public static final Pose2d kBlueStartPose = new Pose2d(2.0, 4.05, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedStartPose = new Pose2d(14.49, 4.05, Rotation2d.fromDegrees(180));

        // Test başlangıç pozisyonu (her iki ittifak için)
        public static final Pose2d kTestStartPose = new Pose2d(8.245, 4.05, Rotation2d.fromDegrees(0));

        // === ALLIANCE ZONE SINIRLARI ===
        // İttifak alanı içinde atış yapılabilir
        // Blue Alliance: Sahanın sol tarafı (X < 4.0m)
        // Red Alliance: Sahanın sağ tarafı (X > 12.49m)
        public static final double kBlueAllianceZoneMaxX = 4.0;
        public static final double kRedAllianceZoneMinX = 12.49;

        // === FEEDING (TOP BESLEME) HEDEFLERİ ===
        // İttifak alanı dışındayken, topları bu noktaya besleme yapılır
        // İttifak alanının giriş noktası (ortası)
        public static final Translation2d kBlueFeedingTarget = new Translation2d(3.5, 4.05);
        public static final Translation2d kRedFeedingTarget = new Translation2d(12.99, 4.05);

        // === ATIŞ PARAMETRELERİ ===
        public static final double kIdealShootingDistanceMeters = 3.0;

        // === DEPOT POZİSYONLARI (Fuel alma noktaları) ===
        public static final Pose2d kBlueDepotPose = new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0));
        public static final Pose2d kRedDepotPose = new Pose2d(14.99, 1.0, Rotation2d.fromDegrees(180));

        // === OUTPOST POZİSYONLARI (Human Player) ===
        public static final Pose2d kBlueOutpostPose = new Pose2d(15.0, 7.0, Rotation2d.fromDegrees(180));
        public static final Pose2d kRedOutpostPose = new Pose2d(1.49, 7.0, Rotation2d.fromDegrees(0));

        // === TOWER POZİSYONLARI (Tırmanma yapısı) ===
        public static final Translation2d kTowerCenter = new Translation2d(8.245, 4.05);
        public static final double kTowerClimbHeightMeters = 1.5;

        public static final Pose2d[] kTowerClimbPoses = new Pose2d[] {
                        new Pose2d(kTowerCenter.getX() - 1.0, kTowerCenter.getY(), Rotation2d.fromDegrees(0)),
                        new Pose2d(kTowerCenter.getX(), kTowerCenter.getY(), Rotation2d.fromDegrees(0)),
                        new Pose2d(kTowerCenter.getX() + 1.0, kTowerCenter.getY(), Rotation2d.fromDegrees(0))
        };

        // === SOURCE/DEPOT POSITIONS ===
        public static final Pose2d[] kSourcePoses = new Pose2d[] {
                        kBlueDepotPose,
                        kBlueOutpostPose
        };

        // === FUEL ÖZELLİKLERİ ===
        public static final double kFuelDiameterMeters = 0.15;

        // === YASAK BÖLGELER (Keep-Out Zones) ===
        // Robotun girmemesi gereken alanların merkez noktaları
        public static final List<Translation2d> kKeepOutZones = new ArrayList<>();

        static {
                // Örnek: Merkez Yapı (Tower) etrafı
                // kKeepOutZones.add(kTowerCenter);
        }

        private FieldConstants() {
        }
}

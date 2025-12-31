package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Saha üzerindeki sabit pozisyonlar ve önemli noktalar.
 * FRC 2024/2025 sahası: 16.54m x 8.02m
 */
public final class FieldConstants {

    // === SAHA ÖLÇÜLERİ ===
    public static final double kFieldLengthMeters = 16.54;
    public static final double kFieldWidthMeters = 8.02;

    // === SPEAKER POZİSYONLARI ===
    // Speaker'lar sahanın iki ucunda, ortada (Y=5.5) konumlu
    public static final Pose2d kBlueSpeakerPose = new Pose2d(0.0, 5.5, Rotation2d.fromDegrees(0));
    public static final Pose2d kRedSpeakerPose = new Pose2d(16.54, 5.5, Rotation2d.fromDegrees(180));

    // === AMP POZİSYONLARI ===
    public static final Pose2d kBlueAmpPose = new Pose2d(1.8, 7.5, Rotation2d.fromDegrees(90));
    public static final Pose2d kRedAmpPose = new Pose2d(14.7, 7.5, Rotation2d.fromDegrees(90));

    // === ATIŞ PARAMETRELERİ ===
    /** İdeal atış mesafesi (Speaker'dan uzaklık) */
    public static final double kIdealShootingDistanceMeters = 3.0;

    // === SOURCE POZİSYONLARI ===
    public static final Pose2d kBlueSourcePose = new Pose2d(15.5, 1.0, Rotation2d.fromDegrees(180));
    public static final Pose2d kRedSourcePose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0));

    // === STAGE POZİSYONLARI ===
    public static final Translation2d kBlueStageCenter = new Translation2d(4.5, 4.0);
    public static final Translation2d kRedStageCenter = new Translation2d(12.0, 4.0);

    private FieldConstants() {
        // Utility class - instantiation prevented
    }
}

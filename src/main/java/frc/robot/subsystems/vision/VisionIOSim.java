package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Supplier;

import frc.robot.constants.VisionConstants;

/**
 * Limelight simülatörü.
 * 
 * <p>
 * Bu sınıf simülasyonda AprilTag tespitini taklit eder.
 * Robot pozisyonunu alarak, görüş alanındaki tag'leri tespit eder
 * ve gerçekçi bir pose tahmini sağlar.
 * </p>
 * 
 * <p>
 * AprilTag pozisyonları WPILib resmi {@link AprilTagFieldLayout} API'si
 * üzerinden yüklenir - her yıl için doğru pozisyonlar otomatik olarak gelir.
 * </p>
 * 
 * @author FRC Team
 */
public class VisionIOSim implements VisionIO {

    // --- KAMERA PARAMETRELERİ ---
    /** Kameranın yatay görüş açısı (derece) - Limelight 3: 63.3° */
    private static final double CAMERA_FOV_HORIZONTAL_DEG = 63.3;

    /** Minimum algılama mesafesi (metre) */
    private static final double MIN_DETECTION_DISTANCE = 0.5;

    /** Maksimum algılama mesafesi (metre) */
    private static final double MAX_DETECTION_DISTANCE = 6.0;

    /** Pose tahmini gürültüsü (metre) - mesafeyle orantılı artacak */
    private static final double BASE_NOISE_METERS = 0.02;

    // NOT: Açısal gürültü yok - robot açısı NavX'ten geliyor, vision sadece X/Y
    // için

    // --- APRIL TAG SAHA DÜZENİ ---
    // WPILib resmi AprilTag saha düzeni API'si kullanılıyor
    // Mevcut yıla ait tüm tag pozisyonları otomatik yüklenir
    private static final AprilTagFieldLayout FIELD_LAYOUT;

    static {
        // 2025 Reefscape saha düzenini yükle (kaynaklı çerçeve versiyonu)
        // Alternatif: k2025ReefscapeAndyMark (AndyMark çerçevesi için)
        FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    /**
     * Yüklü AprilTag saha düzenini döndürür.
     * Diğer sınıflar (örn: DriveToNearestTagCommand) bu metodu kullanabilir.
     */
    public static AprilTagFieldLayout getFieldLayout() {
        return FIELD_LAYOUT;
    }

    private volatile Supplier<Pose2d> poseSupplier;
    private final Random random = new Random();
    private int currentPipeline = VisionConstants.kAprilTagPipelineIndex;

    /**
     * VisionIOSim constructor - pose supplier'sız.
     * setPoseSupplier ile sonradan ayarlanmalı.
     */
    public VisionIOSim() {
        this.poseSupplier = () -> new Pose2d();
    }

    /**
     * VisionIOSim constructor.
     * 
     * @param poseSupplier Robot'un mevcut pozisyonunu sağlayan supplier
     */
    public VisionIOSim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    /**
     * Pose supplier'ı ayarlar.
     * DriveSubsystem oluşturulduktan sonra çağrılmalı.
     * 
     * @param poseSupplier Robot'un mevcut pozisyonunu sağlayan supplier
     */
    public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }
    
    @Override
    public void setPipeline(int pipelineIndex) {
        this.currentPipeline = pipelineIndex;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Varsayılan değerler
        inputs.hasGamePiece = false;
        inputs.gamePieceYaw = 0.0;
        
        // Eğer AprilTag Pipeline seçili değilse, AprilTag aramayı atla
        if (currentPipeline != VisionConstants.kAprilTagPipelineIndex) {
            inputs.hasTarget = false;
            inputs.tagCount = 0;
            inputs.avgTagDist = 0.0;
            return;
        }

        Pose2d robotPose = poseSupplier.get();

        // Kamera pozisyonunu hesapla (robot merkezinden offset)
        double cameraX = robotPose.getX() +
                VisionConstants.kCameraXOffset * robotPose.getRotation().getCos() -
                VisionConstants.kCameraYOffset * robotPose.getRotation().getSin();
        double cameraY = robotPose.getY() +
                VisionConstants.kCameraXOffset * robotPose.getRotation().getSin() +
                VisionConstants.kCameraYOffset * robotPose.getRotation().getCos();
        double cameraYaw = robotPose.getRotation().getRadians();

        // Görülebilir tagleri bul
        List<VisibleTag> visibleTags = new ArrayList<>();

        // Resmi API'den tüm tag'leri al
        for (AprilTag tag : FIELD_LAYOUT.getTags()) {
            Pose3d tagPose = tag.pose;

            // Tag'e olan mesafe
            Translation2d tagXY = new Translation2d(tagPose.getX(), tagPose.getY());
            Translation2d cameraXY = new Translation2d(cameraX, cameraY);
            double distance = tagXY.minus(cameraXY).getNorm();

            // Mesafe kontrolü
            if (distance < MIN_DETECTION_DISTANCE || distance > MAX_DETECTION_DISTANCE) {
                continue;
            }

            // Açı kontrolü (tag kameranın FOV'unda mı?)
            double angleToTag = Math.atan2(
                    tagPose.getY() - cameraY,
                    tagPose.getX() - cameraX);
            double relativeAngle = normalizeAngle(angleToTag - cameraYaw);

            // FOV kontrolü (yatay)
            double halfFovRad = Math.toRadians(CAMERA_FOV_HORIZONTAL_DEG / 2);
            if (Math.abs(relativeAngle) > halfFovRad) {
                continue;
            }

            // Tag görünür! Listeye ekle
            visibleTags.add(new VisibleTag(tag.ID, tagPose, distance));
        }

        // Sonuçları güncelle
        if (!visibleTags.isEmpty()) {
            inputs.hasTarget = true;
            inputs.tagCount = visibleTags.size();

            // Ortalama mesafe hesapla
            double totalDist = 0;
            for (VisibleTag vt : visibleTags) {
                totalDist += vt.distance;
            }
            inputs.avgTagDist = totalDist / visibleTags.size();

            // Pose tahmini: Gerçek pose + X/Y gürültüsü
            // Gürültü mesafeyle orantılı artar (uzak tag = daha fazla hata)
            double noiseScale = BASE_NOISE_METERS * (1 + inputs.avgTagDist / 2);
            double xNoise = (random.nextDouble() - 0.5) * 2 * noiseScale;
            double yNoise = (random.nextDouble() - 0.5) * 2 * noiseScale;
            // NOT: Açısal gürültü yok - açı NavX'ten geliyor

            inputs.estimatedPose = new Pose2d(
                    robotPose.getX() + xNoise,
                    robotPose.getY() + yNoise,
                    robotPose.getRotation()); // Orijinal açı korunuyor

            // Timestamp: Şu anki zaman - küçük gecikme (kamera latency simülasyonu)
            inputs.timestamp = Timer.getFPGATimestamp() - 0.02; // 20ms latency
        } else {
            // Tag görünmüyor
            inputs.hasTarget = false;
            inputs.tagCount = 0;
            inputs.avgTagDist = 0.0;
        }
    }

    /**
     * Açıyı -PI ile PI arasına normalize eder.
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle < -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Görünen tag bilgisini tutan yardımcı record.
     */
    private record VisibleTag(int id, Pose3d pose, double distance) {
    }
}

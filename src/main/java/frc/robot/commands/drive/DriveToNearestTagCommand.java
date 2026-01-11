package frc.robot.commands.drive;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionIOSim;

/**
 * En yakın AprilTag'e otomatik giden komut.
 * 
 * <p>
 * Robot mevcut konumuna en yakın AprilTag'i bulur ve
 * PID kontrolü ile o noktaya gider. Tag'in önünde
 * belirli bir mesafede durur.
 * </p>
 * 
 * <p>
 * AprilTag pozisyonları WPILib resmi {@link AprilTagFieldLayout} API'si
 * üzerinden alınır - her yıl için doğru pozisyonlar otomatik gelir.
 * </p>
 * 
 * @author FRC Team
 */
public class DriveToNearestTagCommand extends Command {

    private final DriveSubsystem driveSubsystem;

    // PID Controllers
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    // Tag'e ne kadar yakın duracak (metre) - robot bumper'ı dikkate alınarak
    private static final double STANDOFF_DISTANCE = 0.5;

    // Toleranslar
    private static final double POSITION_TOLERANCE = 0.1; // metre
    private static final double ROTATION_TOLERANCE = 0.05; // radyan (~3 derece)

    // Hedef pozisyon (initialize'da hesaplanacak)
    private Pose2d targetPose;

    // Resmi AprilTag saha düzeni (VisionIOSim'den paylaşılıyor)
    private static final AprilTagFieldLayout FIELD_LAYOUT = VisionIOSim.getFieldLayout();

    public DriveToNearestTagCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        // PID ayarları (daha agresif)
        xController = new PIDController(3.0, 0.0, 0.2);
        yController = new PIDController(3.0, 0.0, 0.2);
        rotController = new PIDController(4.0, 0.0, 0.2);

        // Rotation controller için continuous input
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        // Toleranslar
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);
        rotController.setTolerance(ROTATION_TOLERANCE);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // PID'leri sıfırla
        xController.reset();
        yController.reset();
        rotController.reset();

        // En yakın tag'i bul
        targetPose = findNearestTagPose();

        System.out.println("[DriveToNearestTag] Hedef: " + targetPose);
    }

    /**
     * En yakın tag'in önündeki hedef pozisyonu hesaplar.
     */
    private Pose2d findNearestTagPose() {
        Pose2d robotPose = driveSubsystem.getPose();
        Translation2d robotXY = robotPose.getTranslation();

        AprilTag nearestTag = null;
        double nearestDist = Double.MAX_VALUE;

        // Resmi API'den tüm tag'leri al
        for (AprilTag tag : FIELD_LAYOUT.getTags()) {
            Translation2d tagXY = new Translation2d(tag.pose.getX(), tag.pose.getY());
            double dist = tagXY.minus(robotXY).getNorm();

            if (dist < nearestDist) {
                nearestDist = dist;
                nearestTag = tag;
            }
        }

        if (nearestTag == null) {
            // Fallback: mevcut pozisyon
            return robotPose;
        }

        // Tag'in 3D pose'undan 2D rotation al
        Pose3d tagPose3d = nearestTag.pose;
        double tagYaw = tagPose3d.getRotation().getZ(); // Tag'in baktığı yön (Z ekseni etrafında)

        // Tag'in önünde duracak pozisyon
        // Tag'in baktığı yönde konumlan (tag dışarı bakıyor, robot da dışarıda olmalı)
        double targetX = tagPose3d.getX() + STANDOFF_DISTANCE * Math.cos(tagYaw);
        double targetY = tagPose3d.getY() + STANDOFF_DISTANCE * Math.sin(tagYaw);

        // Robotun tag'e bakması için açı (tag'in yönünün tersi)
        Rotation2d targetRot = Rotation2d.fromRadians(tagYaw + Math.PI);

        System.out.println("[DriveToNearestTag] En yakın tag ID: " + nearestTag.ID +
                " @ (" + String.format("%.2f", tagPose3d.getX()) +
                ", " + String.format("%.2f", tagPose3d.getY()) + ")");

        return new Pose2d(targetX, targetY, targetRot);
    }

    @Override
    public void execute() {
        if (targetPose == null)
            return;

        Pose2d currentPose = driveSubsystem.getPose();

        // Hata hesapla
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        // Hız limitleri
        xSpeed = clamp(xSpeed, -3.0, 3.0);
        ySpeed = clamp(ySpeed, -3.0, 3.0);
        rotSpeed = clamp(rotSpeed, -3.0, 3.0);

        // Field-relative sürüş
        driveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, currentPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, 0));
        if (!interrupted) {
            System.out.println("[DriveToNearestTag] Hedefe ulaşıldı!");
        } else {
            System.out.println("[DriveToNearestTag] İptal edildi.");
        }
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

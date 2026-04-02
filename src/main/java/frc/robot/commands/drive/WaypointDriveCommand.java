package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.TunableNumber;
import java.util.List;

/**
 * WaypointDriveCommand: Belirlenen noktalar (poses) arasında durmadan geçiş yapan komut.
 * "Automatic Curve" desteği ve "Forbidden Zone" koruması içerir.
 */
public class WaypointDriveCommand extends Command {

    /**
     * Waypoint yapısı: Konum, geçiş toleransı ve ulaşıldığında çalışacak komut.
     */
    public static class Waypoint {
        public final Pose2d pose;
        public final double passingTolerance;
        public final Command onReached;

        public Waypoint(Pose2d pose, double passingTolerance, Command onReached) {
            this.pose = pose;
            this.passingTolerance = passingTolerance;
            this.onReached = onReached;
        }

        public Waypoint(Pose2d pose, double passingTolerance) {
            this(pose, passingTolerance, null);
        }
    }

    private final DriveSubsystem driveSubsystem;
    private final List<Waypoint> waypoints;
    private int currentIndex = 0;

    // PID Kontrolcüleri
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;

    // Tunable PID Değerleri
    private static final TunableNumber kPX = new TunableNumber("SimpleDrive", "kP_X", 4.0);
    private static final TunableNumber kIX = new TunableNumber("SimpleDrive", "kI_X", 0.0);
    private static final TunableNumber kDX = new TunableNumber("SimpleDrive", "kD_X", 0.1);

    private static final TunableNumber kPY = new TunableNumber("SimpleDrive", "kP_Y", 4.0);
    private static final TunableNumber kIY = new TunableNumber("SimpleDrive", "kI_Y", 0.0);
    private static final TunableNumber kDY = new TunableNumber("SimpleDrive", "kD_Y", 0.1);

    private static final TunableNumber kPRot = new TunableNumber("SimpleDrive", "kP_Rot", 3.0);
    private static final TunableNumber kIRot = new TunableNumber("SimpleDrive", "kI_Rot", 0.0);
    private static final TunableNumber kDRot = new TunableNumber("SimpleDrive", "kD_Rot", 0.1);

    // Final Toleranslar
    private static final TunableNumber posTolerance = new TunableNumber("SimpleDrive", "PositionTolerance", 0.2);
    private static final TunableNumber rotTolerance = new TunableNumber("SimpleDrive", "RotationTolerance", 0.08);
    
    // Waypoint Geçiş ve Kavis Ayarları
    private static final TunableNumber defaultPassingTolerance = new TunableNumber("SimpleDrive", "DefaultPassingTolerance", 0.8);
    private static final TunableNumber curveRadius = new TunableNumber("SimpleDrive", "CurveRadius", 1.5);

    /**
     * @param driveSubsystem Sürüş alt sistemi
     * @param waypoints      Takip edilecek noktalar listesi
     */
    public WaypointDriveCommand(DriveSubsystem driveSubsystem, List<Waypoint> waypoints) {
        this.driveSubsystem = driveSubsystem;
        this.waypoints = waypoints;

        // PID ayarları
        xController = new PIDController(kPX.get(), kIX.get(), kDX.get());
        yController = new PIDController(kPY.get(), kIY.get(), kDY.get());
        rotController = new PIDController(kPRot.get(), kIRot.get(), kDRot.get());

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        // Varsayılan toleranslar
        xController.setTolerance(posTolerance.get());
        yController.setTolerance(posTolerance.get());
        rotController.setTolerance(rotTolerance.get());

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentIndex = 0;
        xController.reset();
        yController.reset();
        rotController.reset();
        
        // --- YASAKLI BÖLGE KONTROLÜ (Pre-Scan) ---
        for (int i = 0; i < waypoints.size(); i++) {
            if (FieldConstants.isPoseForbidden(waypoints.get(i).pose)) {
                System.err.println("[WaypointDrive] UYARI: Nokta " + i + " YASAKLI BÖLGEDE! " + waypoints.get(i).pose);
            }
        }
        
        System.out.println("[WaypointDrive] Başlatıldı. Toplam nokta: " + waypoints.size());
    }

    @Override
    public void execute() {
        if (waypoints.isEmpty()) return;

        Pose2d currentPose = driveSubsystem.getPose();
        Waypoint currentWaypoint = waypoints.get(currentIndex);
        double activeTolerance = Math.max(currentWaypoint.passingTolerance, defaultPassingTolerance.get());
        double distanceToCurrent = currentPose.getTranslation().getDistance(currentWaypoint.pose.getTranslation());

        // PID Değerlerini Güncelle
        updatePIDConfigs();

        // --- KAVİS (CURVE) VE HEDEF HESAPLAMA ---
        Pose2d targetPose = currentWaypoint.pose;

        if (currentIndex < waypoints.size() - 1) {
            double radius = curveRadius.get();
            if (distanceToCurrent < radius) {
                Waypoint nextWaypoint = waypoints.get(currentIndex + 1);
                double t = (radius - distanceToCurrent) / (radius - activeTolerance);
                t = MathUtil.clamp(t, 0.0, 1.0);
                targetPose = currentWaypoint.pose.interpolate(nextWaypoint.pose, t);
            }
        }

        // --- HEDEF YASAKLI MI? ---
        // Eğer interpolasyon sonucu hedef yasaklı bir bölgeye girdiyse, hareketi kısıtlamak gerekebilir.
        // Ancak ana hedef (currentWaypoint.pose) zaten yasaklıysa robot duracaktır (DriveSubsystem Safety layer sayesinde).

        // PID Kontrolcüleri ile Hız Hesapla
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotSpeed = rotController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        // Hız Limitleri
        xSpeed = MathUtil.clamp(xSpeed, -3.0, 3.0);
        ySpeed = MathUtil.clamp(ySpeed, -3.0, 3.0);
        rotSpeed = MathUtil.clamp(rotSpeed, -2.0, 2.0);

        // Sürüş
        driveSubsystem.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeed, -ySpeed, -rotSpeed, currentPose.getRotation()));

        // --- NOKTA GEÇİŞ KONTROLÜ ---
        if (currentIndex < waypoints.size() - 1) {
            if (distanceToCurrent < activeTolerance) {
                System.out.println("[WaypointDrive] Nokta " + currentIndex + " geçildi. Bir sonrakine gidiliyor.");
                if (currentWaypoint.onReached != null) {
                    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(currentWaypoint.onReached);
                }
                currentIndex++;
            }
        }

        // Loglama
        org.littletonrobotics.junction.Logger.recordOutput("WaypointDrive/TargetPose", targetPose);
        org.littletonrobotics.junction.Logger.recordOutput("WaypointDrive/CurrentIndex", currentIndex);
        org.littletonrobotics.junction.Logger.recordOutput("WaypointDrive/DistanceToPoint", distanceToCurrent);
    }

    private void updatePIDConfigs() {
        if (kPX.hasChanged() || kIX.hasChanged() || kDX.hasChanged()) {
            xController.setPID(kPX.get(), kIX.get(), kDX.get());
        }
        if (kPY.hasChanged() || kIY.hasChanged() || kDY.hasChanged()) {
            yController.setPID(kPY.get(), kIY.get(), kDY.get());
        }
        if (kPRot.hasChanged() || kIRot.hasChanged() || kDRot.hasChanged()) {
            rotController.setPID(kPRot.get(), kIRot.get(), kDRot.get());
        }
        if (posTolerance.hasChanged()) {
            xController.setTolerance(posTolerance.get());
            yController.setTolerance(posTolerance.get());
        }
        if (rotTolerance.hasChanged()) {
            rotController.setTolerance(rotTolerance.get());
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.runVelocity(new ChassisSpeeds(0, 0, 0));
        System.out.println("[WaypointDrive] Bitti. Interrupted: " + interrupted);
    }

    @Override
    public boolean isFinished() {
        if (waypoints.isEmpty()) return true;
        return currentIndex == waypoints.size() - 1 && 
               xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotController.atSetpoint();
    }
}

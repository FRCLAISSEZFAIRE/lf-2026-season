package frc.robot.constants;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;

public final class DriveConstants {

    // --- PathPlanner Config Loader ---
    private static final JsonNode configJson = loadConfig();

    private static JsonNode loadConfig() {
        try {
            File configFile = new File(Filesystem.getDeployDirectory(), "pathplanner/settings.json");
            return new ObjectMapper().readTree(configFile);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException("Could not load PathPlanner settings.json from deploy directory!", e);
        }
    }

    // Motor ID'ler -> RobotMap'ten alınıyor
    public static final int kFrontLeftDriveID = RobotMap.kFrontLeftDriveID;
    public static final int kFrontLeftTurnID = RobotMap.kFrontLeftTurnID;
    public static final int kFrontRightDriveID = RobotMap.kFrontRightDriveID;
    public static final int kFrontRightTurnID = RobotMap.kFrontRightTurnID;
    public static final int kRearLeftDriveID = RobotMap.kRearLeftDriveID;
    public static final int kRearLeftTurnID = RobotMap.kRearLeftTurnID;
    public static final int kRearRightDriveID = RobotMap.kRearRightDriveID;
    public static final int kRearRightTurnID = RobotMap.kRearRightTurnID;

    // Encoder Offsets (Radyan)
    public static final double kFrontLeftOffsetRad = 0.0;
    public static final double kFrontRightOffsetRad = 0.0;
    public static final double kRearLeftOffsetRad = 0.0;
    public static final double kRearRightOffsetRad = 0.0;

    // Kinematik - JSON'dan çekiliyor
    public static final double kTrackWidthMeters = configJson.path("robotTrackwidth").asDouble();
    // PathPlanner "robotLength" genelde bumper dahil uzunluktur. Swerve kinematik için
    // module offsetlerini kullanmak daha doğrudur.
    // Eğer kare ise TrackWidth kullanılabilir veya FL modül X konumundan çekilebilir.
    public static final double kWheelBaseMeters = configJson.path("flModuleX").asDouble() * 2.0; 
    public static final double kWheelDiameterMeters = configJson.path("driveWheelRadius").asDouble() * 2.0;

    // Swerve Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2), // FL
            new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2), // FR
            new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2), // BL
            new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2) // BR
    );

    // Hız Limitleri - JSON'dan çekiliyor
    public static final double kMaxSpeedMetersPerSecond = configJson.path("maxDriveSpeed").asDouble();
    // Açısal hız genelde JSON'da derece/saniye olabilir, kontrol edelim.
    // settings.json: "defaultMaxAngVel": 360.0 (deg/s) ? Yoksa maxAngularVelocity alanı var mı?
    // settings.json içinde "defaultMaxAngVel" var ama bu PathPlanner'ın constraint'i.
    // Fiziksel limit genellikle robotun max hızı / yarıçap ile ilgilidir.
    // Şimdilik eski değeri koruyalım veya hesaplayalım:
    public static final double kMaxAngularSpeedRadPerSec = Math.PI * 2; 

    // Sürüş PID (Basit)
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    // Dönüş PID
    public static final double kTurnP = 5.0;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;

    // Gyro
    public static final boolean kGyroReversed = false;
}
package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class DriveConstants {

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

    // Kinematik
    public static final double kTrackWidthMeters = 0.57;
    public static final double kWheelBaseMeters = 0.57;
    public static final double kWheelDiameterMeters = 0.1016; // 4 inch

    // Swerve Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2), // FL
            new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2), // FR
            new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2), // BL
            new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2) // BR
    );

    // Hız Limitleri
    public static final double kMaxSpeedMetersPerSecond = 4.5;
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
package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // Physical Constants
    public static final double kTrackWidth = Units.metersToInches(0.55); // 0.55m -> ~21.65 inch
    public static final double kWheelBase = Units.metersToInches(0.58); // 0.58m -> ~22.8 inch

    // Derived for Kinematics (Meters)
    public static final double kTrackWidthMeters = Units.inchesToMeters(21.65);
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.83);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2, kTrackWidthMeters / 2), // FL
            new Translation2d(kWheelBaseMeters / 2, -kTrackWidthMeters / 2), // FR
            new Translation2d(-kWheelBaseMeters / 2, kTrackWidthMeters / 2), // BL
            new Translation2d(-kWheelBaseMeters / 2, -kTrackWidthMeters / 2) // BR
    );

    // Motor IDs (Fetched from RobotMap for consistency)
    public static final int kFrontLeftDrivingCanId = RobotMap.kFrontLeftDriveID;
    public static final int kFrontLeftTurningCanId = RobotMap.kFrontLeftTurnID;
    public static final int kFrontRightDrivingCanId = RobotMap.kFrontRightDriveID;
    public static final int kFrontRightTurningCanId = RobotMap.kFrontRightTurnID;
    public static final int kRearLeftDrivingCanId = RobotMap.kRearLeftDriveID;
    public static final int kRearLeftTurningCanId = RobotMap.kRearLeftTurnID;
    public static final int kRearRightDrivingCanId = RobotMap.kRearRightDriveID;
    public static final int kRearRightTurningCanId = RobotMap.kRearRightTurnID;

    // Angular Offsets (Radians) - Using the fixed values (90, 0, 180, -90)
    public static final double kFrontLeftChassisAngularOffset = Math.PI / 2; // 90 deg
    public static final double kFrontRightChassisAngularOffset = 0; // 0 deg
    public static final double kRearLeftChassisAngularOffset = Math.PI; // 180 deg
    public static final double kRearRightChassisAngularOffset = -Math.PI / 2; // -90 deg

    // ===========================================================================
    // MOTOR INVERSION SETTINGS
    // Ters dönen motorlar için true yapın. Robot test edilirken ayarlayın.
    // ===========================================================================
    public static final boolean kFrontLeftDrivingInverted = false; // FL: Normal
    public static final boolean kFrontRightDrivingInverted = true; // FR: INVERTED
    public static final boolean kRearLeftDrivingInverted = true; // RL: INVERTED
    public static final boolean kRearRightDrivingInverted = false; // RR: Normal

    // Speed Limits
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Legacy Compatibility (renaming for existing code)
    public static final double kMaxAngularSpeedRadPerSec = kMaxAngularSpeed;

    public static final boolean kGyroReversed = true;

    // PID Constants (Module PID - defined in Configs.java, these are legacy or
    // unused but kept for compatibility if needed)
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kMaxSpeedMetersPerSecond;

    // Turning PID (Module PID - defined in Configs.java)
    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;

    // Drive PID (For DriveWithAiming / PathPlanner)
    public static final double kDriveP = 0.1;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final double kTurnP = 5.0; // Keeping this for DriveWithAiming
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;

    public static final double kDriveDeadband = 0.05;
}
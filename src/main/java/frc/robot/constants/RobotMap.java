package frc.robot.constants;

/**
 * Tüm motor ve sensör ID'lerini içerir.
 * CAN ID'leri ve DIO portları burada tanımlanır.
 */
public final class RobotMap {

    // ==================== SWERVE DRIVE ====================
    // Front Left Module
    public static final int kFrontLeftDriveID = 1;
    public static final int kFrontLeftTurnID = 2;

    // Front Right Module
    public static final int kFrontRightDriveID = 3;
    public static final int kFrontRightTurnID = 4;

    // Rear Left Module
    public static final int kRearLeftDriveID = 5;
    public static final int kRearLeftTurnID = 6;

    // Rear Right Module
    public static final int kRearRightDriveID = 7;
    public static final int kRearRightTurnID = 8;

    // ==================== MECHANISMS (10-19) ====================
    // Intake
    public static final int kIntakeMotorID = 10; // NEO
    public static final int kIntakePivotMotorID = 11; // NEO - Pivot

    // Shooter & Feeder
    public static final int kShooterMasterID = 12;
    public static final int kShooterFollowerID = 13;
    public static final int kFeederMotorID = 14; // NEO (User Request ID: 14)
    public static final int kTurretMotorID = 15; // NEO

    // Lift / Climber (20-29)
    public static final int kLiftLeftMotorID = 20;
    public static final int kLiftRightMotorID = 21;

    // ==================== DIO PORTS ====================
    // MZ80 Sensörler
    // MZ80 Sensörler (Feeder)
    public static final int kFeederSensorBottomID = 0;
    public static final int kFeederSensorTopID = 1;

    // ==================== PWM PORTS ====================
    // LED
    public static final int kLEDPort = 0;
    public static final int kLEDLength = 60; // LED strip uzunluğu

    // ==================== CONTROLLERS ====================
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
}

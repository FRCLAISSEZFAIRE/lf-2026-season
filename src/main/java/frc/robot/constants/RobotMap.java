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
    public static final int kIntakeMotorID = 10; // Kraken X60 (Velocity)
    public static final int kIntakePivotMotorID = 11; // NEO

    // Shooter
    public static final int kShooterMasterID = 12; // Kraken X60 (Flywheel)
    public static final int kTurretMotorID = 14; // NEO
    public static final int kHoodMotorID = 15; // NEO 550

    // Feeder (Indexer + Kicker)
    public static final int kIndexerMotorID = 16; // NEO v1.1 (Indexer)
    public static final int kKickerMotorID = 17;  // NEO v1.1 (Kicker)

    // ==================== LIFT / CLIMBER (20-29) ====================
    public static final int kClimberLeftMotorID = 20; // NEO
    public static final int kClimberRightMotorID = 21; // NEO

    // ==================== PWM PORTS ====================
    // LED
    public static final int kLEDPort = 9;
    public static final int kLEDLength = 11;

    // ==================== CONTROLLERS ====================
    public static final int kDriverControllerPort = 0;
}
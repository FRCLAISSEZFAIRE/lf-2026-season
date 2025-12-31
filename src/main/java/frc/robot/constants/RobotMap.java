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

    // ==================== MECHANISMS ====================
    // Intake
    public static final int kIntakeMotorID = 10;

    // Shooter
    public static final int kShooterMasterID = 11;
    public static final int kShooterFollowerID = 12;
    public static final int kTurretMotorID = 13;
    public static final int kHoodMotorID = 14; // Atış açısı (pitch)

    // Elevator
    public static final int kElevatorMotorID = 20;

    // Arm
    public static final int kArmMotorID = 21;

    // Grabber
    public static final int kGrabberMotorID = 22;

    // Climber
    public static final int kClimberLeftMotorID = 23;
    public static final int kClimberRightMotorID = 24;

    // Wrist (Arm ucunda döner eklem)
    public static final int kWristMotorID = 25;

    // Feeder (Intake → Shooter arası)
    public static final int kFeederMotorID = 26;

    // ==================== DIO PORTS ====================
    // MZ80 Sensörler
    public static final int kIntakeMZ80Port = 0;
    public static final int kShooterMZ80Port = 1;

    // ==================== PWM PORTS ====================
    // LED
    public static final int kLEDPort = 0;
    public static final int kLEDLength = 60; // LED strip uzunluğu

    // ==================== CONTROLLERS ====================
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
}

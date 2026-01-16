package frc.robot.constants;

/**
 * Climber alt sistemi için sabitler.
 * 2 Kraken X60 motor ile Motion Magic position control.
 */
public final class ClimberConstants {

    // --- MOTOR IDs ---
    public static final int kLeftMotorID = 20;
    public static final int kRightMotorID = 23;

    // --- SOFT LIMITS (Rotor rotasyonu cinsinden) ---
    public static final double kForwardSoftLimit = 100.0;
    public static final double kReverseSoftLimit = 0.0;

    // --- DIO PORTS ---
    public static final int kSeatSensorDIO = 4; // Tırmanma kancası sensörü (Limit switch)

    // --- MOTION MAGIC PID (Slot 0) ---
    public static final double kClimberS = 0.25; // Static friction
    public static final double kClimberV = 0.12; // Velocity feedforward
    public static final double kClimberA = 0.01; // Acceleration feedforward
    public static final double kClimberP = 4.8; // Proportional gain
    public static final double kClimberI = 0.0; // Integral gain
    public static final double kClimberD = 0.1; // Derivative gain

    // --- MOTION MAGIC PROFILING ---
    public static final double kClimberMMCV = 80.0; // Cruise velocity (rps)
    public static final double kClimberMMA = 160.0; // Acceleration (rps/s)
    public static final double kClimberMMJ = 1600.0; // Jerk (rps/s/s)

    // --- CLIMB POZISYONLARI (Rotor rotasyonu) ---
    public static final double kHomePosition = 0.0; // En alt (başlangıç)
    public static final double kClimbExtendPosition = 95.0; // Tam uzatma (kanca takma)
    public static final double kClimbRetractPosition = 10.0; // Kendini çekme (tırmanma)
    public static final double kClimbHoldPosition = 30.0; // Tutma pozisyonu

    // --- MANUEL KONTROL HIZLARI (rps) ---
    public static final double kManualUpVelocity = 30.0;
    public static final double kManualDownVelocity = -30.0;

    // --- CURRENT LIMITS ---
    public static final double kSupplyCurrentLimit = 60.0;
    public static final double kStallCurrentThreshold = 50.0;

    // --- TOLERANCE ---
    public static final double kPositionTolerance = 1.0;

    // --- CAN OPTIMIZATION ---
    public static final int kPositionUpdateHz = 50;
    public static final int kVelocityUpdateHz = 50;
    public static final int kCurrentUpdateHz = 20;
    public static final int kTemperatureUpdateHz = 4;

    // --- SAFETY THRESHOLDS ---
    public static final double kTiltWarningThreshold = 15.0; // Derece
    public static final double kTiltDangerThreshold = 25.0; // Derece
}

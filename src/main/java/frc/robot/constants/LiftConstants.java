package frc.robot.constants;

/**
 * Lift alt sistemi için sabitler.
 * Elevator + Climber birleşik sistem.
 * 2 Kraken X60 motor ile Motion Magic position control.
 */
public final class LiftConstants {

    // --- MOTOR IDs ---
    public static final int kLeftMotorID = 20; // Eski Elevator ID
    public static final int kRightMotorID = 23; // Eski Climber Sol ID

    // --- SOFT LIMITS (Rotor rotasyonu cinsinden) ---
    public static final double kForwardSoftLimit = 100.0;
    public static final double kReverseSoftLimit = 0.0;

    // --- MOTION MAGIC PID (Slot 0) ---
    public static final double kLiftS = 0.25; // Static friction
    public static final double kLiftV = 0.12; // Velocity feedforward
    public static final double kLiftA = 0.01; // Acceleration feedforward
    public static final double kLiftP = 4.8; // Proportional gain
    public static final double kLiftI = 0.0; // Integral gain
    public static final double kLiftD = 0.1; // Derivative gain

    // --- MOTION MAGIC PROFILING ---
    public static final double kLiftMMCV = 80.0; // Cruise velocity (rps)
    public static final double kLiftMMA = 160.0; // Acceleration (rps/s)
    public static final double kLiftMMJ = 1600.0; // Jerk (rps/s/s)

    // --- ELEVATOR POZISYONLARI (Rotor rotasyonu) ---
    public static final double kLevel0Position = 0.0; // Home
    public static final double kLevel1Position = 20.0; // Düşük skor
    public static final double kLevel2Position = 45.0; // Orta skor
    public static final double kLevel3Position = 75.0; // Yüksek skor
    public static final double kIntakePosition = 5.0; // Intake pozisyonu

    // --- CLIMB POZISYONLARI (Rotor rotasyonu) ---
    public static final double kClimbExtendPosition = 95.0; // Tam uzatma (tırmanma hazırlık)
    public static final double kClimbRetractPosition = 10.0; // Geri çekme (tırmanma)
    public static final double kClimbHoldPosition = 30.0; // Tutma pozisyonu

    // --- MANUEL KONTROL HIZLARI (rps) ---
    public static final double kManualUpVelocity = 30.0;
    public static final double kManualDownVelocity = -30.0;

    // --- CURRENT LIMITS ---
    public static final double kSupplyCurrentLimit = 60.0; // Kraken yüksek akım çekebilir
    public static final double kStallCurrentThreshold = 50.0;

    // --- TOLERANCE ---
    public static final double kPositionTolerance = 1.0;

    // --- CAN OPTIMIZATION (Update frequencies in Hz) ---
    public static final int kPositionUpdateHz = 50; // Default 250Hz → 50Hz
    public static final int kVelocityUpdateHz = 50;
    public static final int kCurrentUpdateHz = 20;
    public static final int kTemperatureUpdateHz = 4;

    // --- SAFETY THRESHOLDS ---
    public static final double kTiltWarningThreshold = 15.0; // Derece
    public static final double kTiltDangerThreshold = 25.0; // Derece
}

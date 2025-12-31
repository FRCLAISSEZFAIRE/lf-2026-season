package frc.robot.constants;

/**
 * Elevator alt sistemi için sabitler.
 * Motion Magic ve soft limit ayarları.
 */
public final class ElevatorConstants {

    // --- MOTOR ID (RobotMap'ten) ---
    public static final int kElevatorMotorID = RobotMap.kElevatorMotorID;

    // --- SOFT LIMITS (Rotor rotasyonu cinsinden) ---
    public static final double kForwardSoftLimit = 100.0;
    public static final double kReverseSoftLimit = 0.0;

    // --- MOTION MAGIC PID (Slot 0) ---
    public static final double kElevatorS = 0.25;
    public static final double kElevatorV = 0.12;
    public static final double kElevatorA = 0.01;
    public static final double kElevatorP = 4.8;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.1;

    // --- MOTION MAGIC PROFILING ---
    public static final double kElevatorMMCV = 80.0;
    public static final double kElevatorMMA = 160.0;
    public static final double kElevatorMMJ = 1600.0;

    // --- LEVEL POZISYONLARI (Rotor rotasyonu) ---
    public static final double kLevel0Position = 0.0;
    public static final double kLevel1Position = 20.0;
    public static final double kLevel2Position = 45.0;
    public static final double kLevel3Position = 75.0;
    public static final double kIntakePosition = 5.0;

    // --- MANUEL KONTROL HIZLARI (rps) ---
    public static final double kManualUpVelocity = 30.0;
    public static final double kManualDownVelocity = -30.0;
    public static final double kHoldVelocity = 0.0;

    // --- TOLERANCE ---
    public static final double kPositionTolerance = 1.0;
}

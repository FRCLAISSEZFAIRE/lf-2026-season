package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ModuleConstants {
    
    // --- DONANIM YAPILANDIRMASI ---
    public static final int kDrivingMotorPinionTeeth = 13; // 13 Dişli Pinyon (Vortex)
    public static final boolean kTurningEncoderInverted = true;

    // --- ÖLÇÜLER ---
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // --- VORTEX & MAXSWERVE HESAPLAMALARI ---
    // Vortex Free Speed: ~6784 RPM
    public static final double kVortexFreeSpeedRpm = 6784.0;
    public static final double kDrivingMotorFreeSpeedRps = kVortexFreeSpeedRpm / 60.0;

    // Sürüş Dişli Oranı (13T Pinyon için hesap)
    // 45 ve 22: Spur Gear dişlileri, 15: Bevel Pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    
    // Dönüşüm Faktörleri (Encoder -> Metre)
    public static final double kDrivingEncoderPositionFactor = kWheelCircumferenceMeters / kDrivingMotorReduction;
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0;

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // Radyan
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // Rad/s

    // --- PID & AKIM LİMİTLERİ ---
    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    // FeedForward Hesabı (Vortex için)
    public static final double kDrivingFF = 1.0 / (kDrivingMotorFreeSpeedRps * kDrivingEncoderPositionFactor);

    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;

    public static final int kDrivingMotorCurrentLimit = 60; // Amper (Vortex)
    public static final int kTurningMotorCurrentLimit = 20; // Amper (Neo 550)
}
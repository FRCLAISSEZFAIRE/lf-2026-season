package frc.robot.constants;

import frc.robot.constants.Constants.JoystickType;

public final class OIConstants {
    public static final int kDriverControllerPort = RobotMap.kDriverControllerPort;
    public static final int kOperatorControllerPort = RobotMap.kOperatorControllerPort;
    public static final double kDriveDeadband = 0.1;
    public static final double kDriveSensitivity = 0.8; // Genel sürüş hassasiyeti (0.0 - 1.0)
    public static final double kTurnSensitivity = 0.6; // Dönüş hassasiyeti (0.0 - 1.0)

    // --- JOYSTICK AXIS MAPPING ---
    // Joystick tipine göre farklı axis mapping kullanılır.
    // Xbox: Standart FRC Controller
    // Generic: USB Joystick (farklı axis düzeni olabilir)

    public static final int kDriverLeftXAxis;
    public static final int kDriverLeftYAxis;
    public static final int kDriverRotAxis;

    static {
        // Joystick tipi Constants.java'dan okunur
        JoystickType joystickType = Constants.driverJoystickType;

        // Robot simülasyonda mı ve Mac üzerinde mi?
        boolean isMac = System.getProperty("os.name").toLowerCase().contains("mac");
        boolean isSim = edu.wpi.first.wpilibj.RobotBase.isSimulation();

        if (joystickType == JoystickType.GENERIC) {
            // --- GENERIC JOYSTICK MAPPING ---
            // Çoğu generic joystick: X=0, Y=1, Twist/Z=2
            // System.out.println("[OIConstants] GENERIC Joystick mapping aktif: X=0, Y=1,
            // Rot=2");
            kDriverLeftXAxis = 0;
            kDriverLeftYAxis = 1;
            kDriverRotAxis = 2; // Twist axis (genellikle 2 veya 3)
        } else if (isSim && isMac) {
            // --- XBOX MACOS SİMÜLASYON MAPPING ---
            // MacOS'ta Xbox simülasyonu: LeftX=0, LeftY=1, RightX=2
            // System.out.println("[OIConstants] Xbox MacOS Simülasyon mapping aktif:
            // Rotasyon Axis 2");
            kDriverLeftXAxis = 0;
            kDriverLeftYAxis = 1;
            kDriverRotAxis = 2; // Right Stick X
        } else {
            // --- XBOX ROBORIO / WINDOWS STANDART MAPPING ---
            // Standart: LeftX=0, LeftY=1, LT=2, RT=3, RightX=4, RightY=5
            // System.out.println("[OIConstants] Xbox Standart (RoboRIO) mapping aktif:
            // Rotasyon Axis 4");
            kDriverLeftXAxis = 0; // XboxController.Axis.kLeftX.value
            kDriverLeftYAxis = 1; // XboxController.Axis.kLeftY.value
            kDriverRotAxis = 4; // XboxController.Axis.kRightX.value
        }
    }
}

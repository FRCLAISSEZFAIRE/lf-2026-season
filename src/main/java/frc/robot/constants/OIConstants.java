package frc.robot.constants;

public final class OIConstants {
    public static final int kDriverControllerPort = RobotMap.kDriverControllerPort;
    public static final int kOperatorControllerPort = RobotMap.kOperatorControllerPort;
    public static final double kDriveDeadband = 0.1;

    // --- XBOX AXIS MAPPING AUTOMATION ---
    // RoboRIO/Windows (Standart) vs MacOS (Simülasyon) farklı mapping kullanır.
    // Bu blok otomatik olarak doğru eksenleri seçer.

    public static final int kDriverLeftXAxis;
    public static final int kDriverLeftYAxis;
    public static final int kDriverRotAxis;

    static {
        // Robot simülasyonda mı ve Mac üzerinde mi?
        boolean isMac = System.getProperty("os.name").toLowerCase().contains("mac");
        boolean isSim = edu.wpi.first.wpilibj.RobotBase.isSimulation();

        if (isSim && isMac) {
            // --- MACOS SİMÜLASYON MAPPING ---
            // Genellikle: LeftX=0, LeftY=1, RightX=2, RightY=3, LT=4, RT=5
            System.out.println("[OIConstants] MacOS Simülasyon mapping'i aktif: Rotasyon Axis 2");
            kDriverLeftXAxis = 0;
            kDriverLeftYAxis = 1;
            kDriverRotAxis = 2; // Right Stick X
        } else {
            // --- ROBORIO / WINDOWS STANDART MAPPING ---
            // Standart: LeftX=0, LeftY=1, LT=2, RT=3, RightX=4, RightY=5
            System.out.println("[OIConstants] Standart (RoboRIO) mapping'i aktif: Rotasyon Axis 4");
            kDriverLeftXAxis = 0; // XboxController.Axis.kLeftX.value
            kDriverLeftYAxis = 1; // XboxController.Axis.kLeftY.value
            kDriverRotAxis = 4; // XboxController.Axis.kRightX.value
        }
    }
}

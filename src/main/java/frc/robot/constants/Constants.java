package frc.robot.constants;

public final class Constants {
    // Robotun çalışma modu
    public static final Mode currentMode = Mode.REAL;

    public enum Mode {
        REAL, // Gerçek Robot
        REPLAY, // Log Dosyası Oynatma
        SIM // Fizik Simülasyonu
    }

    // Robot döngü süresi (Genelde 20ms = 0.02s)
    public static final double kLoopPeriodSecs = 0.02;

    // Tuning Modu: Açıkken SmartDashboard'dan PID değiştirebilirsin
    public static final boolean tuningMode = true;

    // ==================== JOYSTICK TİPİ ====================
    /**
     * Sürücü joystick tipi seçimi.
     * Test için GENERIC, yarışma için XBOX kullanılabilir.
     */
    public enum JoystickType {
        XBOX, // Xbox Controller (varsayılan)
        GENERIC // Generic USB Joystick
    }

    // Sürücü joystick tipi - TEST İÇİN DEĞİŞTİRİLEBİLİR
    public static final JoystickType driverJoystickType = JoystickType.XBOX;
}
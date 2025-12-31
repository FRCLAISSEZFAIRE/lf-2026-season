package frc.robot.constants;

public final class Constants {
    // Robotun çalışma modu
    public static final Mode currentMode = Mode.SIM;

    public enum Mode {
        REAL, // Gerçek Robot
        REPLAY, // Log Dosyası Oynatma
        SIM // Fizik Simülasyonu
    }

    // Robot döngü süresi (Genelde 20ms = 0.02s)
    public static final double kLoopPeriodSecs = 0.02;

    // Tuning Modu: Açıkken SmartDashboard'dan PID değiştirebilirsin
    public static final boolean tuningMode = true;
}
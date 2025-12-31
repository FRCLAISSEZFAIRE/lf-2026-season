package frc.robot.constants;

/**
 * LED alt sistemi için sabitler.
 * AddressableLED (WS2812B) ayarları.
 */
public final class LEDConstants {

    // --- PORT (RobotMap'ten) ---
    public static final int kLEDPort = RobotMap.kLEDPort;
    public static final int kLEDLength = RobotMap.kLEDLength;

    // --- RENKLER (RGB) ---
    // Off
    public static final int[] kColorOff = { 0, 0, 0 };

    // Takım Rengi (Turuncu - #FF6500)
    public static final int[] kColorTeam = { 255, 101, 0 };

    // Durum Renkleri
    public static final int[] kColorIdle = { 0, 0, 255 }; // Mavi - Bekleme
    public static final int[] kColorIntaking = { 0, 255, 0 }; // Yeşil - Intake
    public static final int[] kColorHasGamePiece = { 255, 165, 0 }; // Turuncu - Parça var
    public static final int[] kColorShooting = { 255, 0, 0 }; // Kırmızı - Atış
    public static final int[] kColorClimbing = { 255, 255, 0 }; // Sarı - Tırmanma
    public static final int[] kColorError = { 255, 0, 255 }; // Mor - Hata

    // Animasyon Hızı
    public static final double kBlinkIntervalSeconds = 0.25;
    public static final double kRainbowSpeedPerSecond = 50.0;
}

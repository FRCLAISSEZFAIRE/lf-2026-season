package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

/**
 * NetworkTables üzerinden runtime'da değiştirilebilen ve KALICI olarak saklanan
 * sayı.
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Doğrudan NetworkTables üzerinden okunabilir/yazılabilir (Elastic,
 * Shuffleboard, Glass ile uyumlu)</li>
 * <li>Değerler RoboRIO flash belleğinde saklanır (Preferences API)</li>
 * <li>Robot kapatılsa bile değerler korunur</li>
 * <li>PID tuning için idealdir</li>
 * </ul>
 * 
 * <h2>Dashboard Erişimi:</h2>
 * <p>
 * Değerler <code>/Tuning/[tabName]/[key]</code> altında yayınlanır.
 * </p>
 * 
 * <h2>Kullanım:</h2>
 * 
 * <pre>
 * TunableNumber kP = new TunableNumber("Shooter", "Turret kP", 0.1);
 * double p = kP.get(); // Dashboard'dan veya Preferences'tan değer okur
 * </pre>
 */
public class TunableNumber {
    private final String fullKey;
    private final double defaultValue;
    private double lastValue;

    /**
     * @param tabName      Grup adı (örn: "Shooter", "Drive")
     * @param key          Değişken adı (örn: "Turret kP")
     * @param defaultValue Varsayılan değer (Preferences'ta yoksa kullanılır)
     */
    public TunableNumber(String tabName, String key, double defaultValue) {
        this.fullKey = tabName + "/" + key;
        this.defaultValue = defaultValue;

        // Preferences'ta değer yoksa default değeri yaz
        if (!Preferences.containsKey(fullKey)) {
            Preferences.initDouble(fullKey, defaultValue);
        }
        
        // İlk değeri oku ve sakla
        this.lastValue = Preferences.getDouble(fullKey, defaultValue);
    }

    /**
     * Mevcut değeri döndürür.
     */
    public double get() {
        double current = Preferences.getDouble(fullKey, lastValue);
        // Dashboard üzerinden değer değiştiyse yerel durumu güncelle
        if (current != lastValue) {
            lastValue = current;
        }
        return current;
    }

    /**
     * Değer değişti mi kontrolü (motor güncelleme için).
     */
    public boolean hasChanged() {
        double current = Preferences.getDouble(fullKey, lastValue);
        if (current != lastValue) {
            lastValue = current;
            return true;
        }
        return false;
    }

    /**
     * Son okunan değeri döndürür.
     */
    public double getLastValue() {
        return lastValue;
    }

    /**
     * Değeri manuel olarak ayarla ve Preferences'a kaydet.
     */
    public void set(double value) {
        lastValue = value;
        Preferences.setDouble(fullKey, value);
    }

    /**
     * Değeri varsayılana sıfırla.
     */
    public void reset() {
        set(defaultValue);
    }

    /**
     * NetworkTables key'ini döndürür.
     */
    public String getKey() {
        return fullKey;
    }

    /**
     * NetworkTables'taki tam yolu döndürür.
     */
    public String getNTPath() {
        return "/Preferences/" + fullKey;
    }
}

package frc.robot.util;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.constants.Constants;

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
    private static final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");

    private final String key;
    private final String fullKey;
    private final double defaultValue;
    private final DoubleEntry ntEntry;
    private double lastValue;

    /**
     * @param tabName      Grup adı (örn: "Shooter", "Drive")
     * @param key          Değişken adı (örn: "Turret kP")
     * @param defaultValue Varsayılan değer (Preferences'ta yoksa kullanılır)
     */
    public TunableNumber(String tabName, String key, double defaultValue) {
        this.key = key;
        this.fullKey = tabName + "/" + key;
        this.defaultValue = defaultValue;

        // Preferences'tan değer oku (varsa), yoksa default kullan
        double storedValue = Preferences.getDouble(fullKey, defaultValue);
        this.lastValue = storedValue;

        // NetworkTables'a entry oluştur
        NetworkTable subTable = tuningTable.getSubTable(tabName);
        ntEntry = subTable.getDoubleTopic(key).getEntry(storedValue);

        // Her zaman NetworkTables'a yayınla (tuningMode kontrolü kaldırıldı)
        ntEntry.set(storedValue);
    }

    /**
     * Mevcut değeri döndürür.
     * Dashboard'dan değer değiştiyse günceller ve Preferences'a kaydeder.
     */
    public double get() {
        double current = ntEntry.get(lastValue);

        // Değer değiştiyse Preferences'a kaydet
        if (current != lastValue) {
            Preferences.setDouble(fullKey, current);
            lastValue = current;
        }

        return current;
    }

    /**
     * Değer değişti mi kontrolü (motor güncelleme için).
     */
    public boolean hasChanged() {
        double current = ntEntry.get(lastValue);
        if (current != lastValue) {
            Preferences.setDouble(fullKey, current);
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
     * Değeri manuel olarak ayarla ve hem NT hem Preferences'a kaydet.
     */
    public void set(double value) {
        lastValue = value;
        Preferences.setDouble(fullKey, value);
        ntEntry.set(value);
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
        return "/Tuning/" + fullKey;
    }
}

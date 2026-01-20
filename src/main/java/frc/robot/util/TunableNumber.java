package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants;

/**
 * NetworkTables üzerinden runtime'da değiştirilebilen ve KALICI olarak saklanan
 * sayı.
 * 
 * <h2>Özellikler:</h2>
 * <ul>
 * <li>Shuffleboard üzerinden değiştirilebilir</li>
 * <li>Değerler RoboRIO flash belleğinde saklanır (Preferences API)</li>
 * <li>Robot kapatılsa bile değerler korunur</li>
 * <li>PID tuning için idealdir</li>
 * </ul>
 * 
 * <h2>Kullanım:</h2>
 * 
 * <pre>
 * TunableNumber kP = new TunableNumber("Shooter", "Turret P", 0.1);
 * // Shuffleboard'dan değiştir, kalıcı olarak kaydedilir
 * double p = kP.get();
 * </pre>
 */
public class TunableNumber {
    private final String key;
    private final String fullKey; // Preferences için benzersiz key
    private final double defaultValue;
    private GenericEntry entry;
    private double lastValue;

    /**
     * @param tabName      Shuffleboard tab adı
     * @param key          Değişken adı
     * @param defaultValue Varsayılan değer (Preferences'ta yoksa kullanılır)
     */
    public TunableNumber(String tabName, String key, double defaultValue) {
        this.key = key;
        this.fullKey = tabName + "/" + key; // Benzersiz key oluştur
        this.defaultValue = defaultValue;

        // Preferences'tan değer oku (varsa), yoksa default kullan
        double storedValue = Preferences.getDouble(fullKey, defaultValue);
        this.lastValue = storedValue;

        // Tuning mode açıkken dashboard'a ekle
        if (Constants.tuningMode) {
            ShuffleboardTab tab = Shuffleboard.getTab(tabName);
            entry = tab.add(key, storedValue)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
        }
    }

    /**
     * Mevcut değeri döndürür.
     * Tuning mode kapalıysa Preferences'tan (kalıcı) değeri okur.
     */
    public double get() {
        if (Constants.tuningMode && entry != null) {
            double current = entry.getDouble(lastValue);
            // Preferences'a kaydet (kalıcı)
            if (current != lastValue) {
                Preferences.setDouble(fullKey, current);
                lastValue = current;
            }
            return current;
        }
        // Tuning mode kapalıysa Preferences'tan oku
        return Preferences.getDouble(fullKey, defaultValue);
    }

    /**
     * Değer değişti mi kontrolü (motor güncelleme için).
     */
    public boolean hasChanged() {
        if (Constants.tuningMode && entry != null) {
            double current = entry.getDouble(lastValue);
            if (current != lastValue) {
                // Preferences'a kaydet
                Preferences.setDouble(fullKey, current);
                lastValue = current;
                return true;
            }
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
        if (Constants.tuningMode && entry != null) {
            entry.setDouble(value);
        }
    }

    /**
     * Preferences'taki değeri sıfırla (default'a döndür).
     */
    public void reset() {
        set(defaultValue);
    }

    /**
     * Preferences key'ini döndürür (debug için).
     */
    public String getKey() {
        return fullKey;
    }
}

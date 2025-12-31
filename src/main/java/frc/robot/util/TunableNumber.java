package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.constants.Constants;

/**
 * NetworkTables üzerinden runtime'da değiştirilebilen sayı.
 * PID tuning için kullanılır.
 */
public class TunableNumber {
    private final String key;
    private final double defaultValue;
    private GenericEntry entry;
    private double lastValue;

    /**
     * @param tabName      Shuffleboard tab adı
     * @param key          Değişken adı
     * @param defaultValue Varsayılan değer
     */
    public TunableNumber(String tabName, String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;

        // Sadece tuning mode açıkken dashboard'a ekle
        if (Constants.tuningMode) {
            ShuffleboardTab tab = Shuffleboard.getTab(tabName);
            entry = tab.add(key, defaultValue)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry();
        }
    }

    /**
     * Mevcut değeri döndürür.
     * Tuning mode kapalıysa varsayılan değeri döndürür.
     */
    public double get() {
        if (Constants.tuningMode && entry != null) {
            return entry.getDouble(defaultValue);
        }
        return defaultValue;
    }

    /**
     * Değer değişti mi kontrolü (PID güncelleme için).
     */
    public boolean hasChanged() {
        if (Constants.tuningMode && entry != null) {
            double current = entry.getDouble(defaultValue);
            if (current != lastValue) {
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
}

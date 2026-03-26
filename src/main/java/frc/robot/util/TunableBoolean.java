package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;

/**
 * NetworkTables üzerinden runtime'da değiştirilebilen ve KALICI olarak saklanan
 * boolean (switch) değeri.
 * Dashboard'da Switch/Checkbox olarak görünmesi için tasarlanmıştır.
 */
public class TunableBoolean {
    private final String fullKey;
    private final boolean defaultValue;
    private boolean lastValue;

    public TunableBoolean(String tabName, String key, boolean defaultValue) {
        this.fullKey = tabName + "/" + key;
        this.defaultValue = defaultValue;

        if (!Preferences.containsKey(fullKey)) {
            Preferences.initBoolean(fullKey, defaultValue);
        }
        
        this.lastValue = Preferences.getBoolean(fullKey, defaultValue);
    }

    public boolean get() {
        boolean current = Preferences.getBoolean(fullKey, lastValue);
        if (current != lastValue) {
            lastValue = current;
        }
        return current;
    }

    public boolean hasChanged() {
        boolean current = Preferences.getBoolean(fullKey, lastValue);
        if (current != lastValue) {
            lastValue = current;
            return true;
        }
        return false;
    }

    public boolean getLastValue() {
        return lastValue;
    }

    public void set(boolean value) {
        lastValue = value;
        Preferences.setBoolean(fullKey, value);
    }

    public void reset() {
        set(defaultValue);
    }

    public String getKey() {
        return fullKey;
    }

    public String getNTPath() {
        return "/Preferences/" + fullKey;
    }
}

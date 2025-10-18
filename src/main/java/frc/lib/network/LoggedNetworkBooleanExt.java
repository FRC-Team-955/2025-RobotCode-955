package frc.lib.network;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class LoggedNetworkBooleanExt extends LoggedNetworkBoolean {
    private boolean lastValue;
    private boolean hasChanged = false;

    public LoggedNetworkBooleanExt(String key, Boolean defaultValue) {
        super("/Tuning/" + removeSlash(key), defaultValue);
        lastValue = defaultValue;
    }

    public boolean hasChanged() {
        return hasChanged;
    }

    @Override
    public void periodic() {
        super.periodic();

        boolean newValue = get();
        if (newValue != lastValue) {
            hasChanged = true;
        } else if (hasChanged) {
            hasChanged = false;
        }
        lastValue = newValue;
    }
}

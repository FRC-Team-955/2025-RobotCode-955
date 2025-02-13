package frc.robot.util.network;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import java.util.HashSet;

public class LoggedNetworkBooleanExt extends LoggedNetworkBoolean {
    private final HashSet<Integer> hashCodesGottenChange = new HashSet<>();
    private boolean lastValue = get();

    public LoggedNetworkBooleanExt(String key) {
        super(key);
    }

    public LoggedNetworkBooleanExt(String key, boolean defaultValue) {
        super(key, defaultValue);
    }

    public boolean hasChanged(int hashCode) {
        if (!hashCodesGottenChange.contains(hashCode)) {
            hashCodesGottenChange.add(hashCode);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        boolean newValue = get();
        if (newValue != lastValue) {
            // Clear all hash codes that have been given the change so that they get the new value
            hashCodesGottenChange.clear();
        }
        lastValue = newValue;
    }
}

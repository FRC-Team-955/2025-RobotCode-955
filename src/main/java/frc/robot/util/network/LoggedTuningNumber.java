package frc.robot.util.network;

import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.HashSet;

public class LoggedTuningNumber extends LoggedNetworkNumber {
    private final HashSet<Integer> hashCodesGottenChange = new HashSet<>();
    private double lastValue = get();

    public LoggedTuningNumber(String key) {
        super(key);
    }

    public LoggedTuningNumber(String key, double defaultValue) {
        super(key, defaultValue);
    }

    public boolean hasChanged(int hashCode) {
        if (Constants.tuningMode && !hashCodesGottenChange.contains(hashCode)) {
            hashCodesGottenChange.add(hashCode);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        if (Constants.tuningMode) {
            super.periodic();

            double newValue = get();
            if (newValue != lastValue) {
                // Clear all hash codes that have been given the change so that they get the new value
                hashCodesGottenChange.clear();
            }
            lastValue = newValue;
        }
    }
}

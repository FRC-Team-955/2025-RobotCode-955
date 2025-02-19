package frc.robot.util.network;

import frc.robot.Constants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.HashSet;

public class LoggedTunableNumber {
    private final LoggedTuningNumberInner inner;
    private double defaultValue;

    public LoggedTunableNumber(String key, double defaultValue) {
        if (Constants.tuningMode) {
            inner = new LoggedTuningNumberInner(key, defaultValue);
        } else {
            this.defaultValue = defaultValue;
        }
    }

    public boolean hasChanged(int hashCode) {
        if (Constants.tuningMode) {
            return inner.hasChanged(hashCode);
        } else {
            return false;
        }
    }

    // FIXME: If you need these methods, you should update hasChanged to work if not in tuning mode
//    /** Updates the default value, which is used if no value in NT is found. */
//    public void setDefault(double defaultValue) {
//        if (Constants.tuningMode) {
//            inner.setDefault(defaultValue);
//        } else {
//            this.valueIfNotTuningMode = defaultValue;
//        }
//    }
//
//    /**
//     * Publishes a new value. Note that the value will not be returned by
//     * {@link #get()} until the next cycle.
//     */
//    public void set(double value) {
//        if (Constants.tuningMode) {
//            inner.set(value);
//        } else {
//            valueIfNotTuningMode = value;
//        }
//    }

    /** Returns the current value. */
    public double get() {
        if (Constants.tuningMode) {
            return inner.get();
        } else {
            return defaultValue;
        }
    }

    private class LoggedTuningNumberInner extends LoggedNetworkNumber {
        private final HashSet<Integer> hashCodesGottenChange = new HashSet<>();
        private double lastValue;
        private boolean hasChangedOnce = false;

        private LoggedTuningNumberInner(String key, double defaultValue) {
            super("/Tuning/" + removeSlash(key), defaultValue);
            lastValue = defaultValue;
        }

        private boolean hasChanged(int hashCode) {
            if (hasChangedOnce && !hashCodesGottenChange.contains(hashCode)) {
                hashCodesGottenChange.add(hashCode);
                return true;
            } else {
                return false;
            }
        }

        @Override
        public void periodic() {
            super.periodic();

            double newValue = get();
            if (newValue != lastValue) {
                // Clear all hash codes that have been given the change so that they get the new value
                hashCodesGottenChange.clear();
                hasChangedOnce = true;
            }
            lastValue = newValue;
        }
    }
}

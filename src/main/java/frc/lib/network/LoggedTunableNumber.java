package frc.lib.network;

import frc.robot.Constants;

public class LoggedTunableNumber {
    private final LoggedNetworkNumberExt inner;
    private final double defaultValue;

    public LoggedTunableNumber(String key, double defaultValue) {
        this.defaultValue = defaultValue;
        if (Constants.tuningMode) {
            inner = new LoggedNetworkNumberExt(key, defaultValue);
        } else {
            inner = null;
        }
    }

    @SuppressWarnings("DataFlowIssue")
    public boolean hasChanged() {
        if (Constants.tuningMode) {
            return inner.hasChanged();
        } else {
            return false;
        }
    }

    @SuppressWarnings("DataFlowIssue")
    public double get() {
        if (Constants.tuningMode) {
            return inner.get();
        } else {
            return defaultValue;
        }
    }
}

package frc.robot.subsystems.leds;

import frc.robot.Constants;

public class LEDConstants {
    public static final int length = 24;

    public static final boolean viewDebug = false;
    public static final double endgameThresholdSeconds = 30;
    public static final double lowBatteryThresholdVolts = 12.0;

    protected static LEDsIO createIO() {
        if (Constants.isReplay) {
            return new LEDsIOSim();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new LEDsIOReal();
            case SIMBOT, ALPHABOT -> new LEDsIOSim();
        };
    }
}
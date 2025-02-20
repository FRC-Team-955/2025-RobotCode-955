package frc.robot.subsystems.climber;

import frc.robot.Constants;

public class ClimberConstants {
    public static final double currentLimitAmps = 20;
    public static final double gearRatio = 128;

    protected static ClimberIO createIO() {
        if (Constants.isReplay) {
            return new ClimberIO();
        }
        return new ClimberIO();
    }
}
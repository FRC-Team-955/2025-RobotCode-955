package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.util.PIDF;

public class ClimberConstants {
    public static final PIDF gains = PIDF.ofP(30);

    public static final double currentLimitAmps = 60;
    public static final double gearRatio = 640;

    protected static ClimberIO createIO() {
        if (Constants.isReplay) {
            return new ClimberIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new ClimberIOTalonFX(10, true, 4, 0.0);
            case SIMBOT -> new ClimberIOSim();
            case ALPHABOT -> new ClimberIO();
        };
    }
}
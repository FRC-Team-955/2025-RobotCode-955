package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureConstants {
    public static final double scoreCoralSettleSeconds = 0.25;

    protected static SuperstructureIO createIO() {
        if (Constants.isReplay) {
            return new SuperstructureIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new SuperstructureIOReal();
            case SIMBOT -> new SuperstructureIOSim();
            case ALPHABOT -> new SuperstructureIO();
        };
    }
}
package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureConstants {
    protected static SuperstructureIO createIO() {
        if (Constants.isReplay) {
            return new SuperstructureIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new SuperstructureIOReal();
            case SIMBOT, ALPHABOT -> new SuperstructureIOSim();
        };
    }

    /** Range for the intake sensor to be triggered */
    public static final double intakeRangeTriggerMeters = 1;
}
package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureConstants {
    protected static final SuperstructureIO io = Constants.isReplay
            ? new SuperstructureIO()
            : switch (Constants.identity) {
        case COMPBOT -> null;
        case SIMBOT, ALPHABOT -> new SuperstructureIOSim();
    };

    /** Range for the intake sensor to be triggered */
    public static final double intakeRangeTriggerMeters = 1;
}
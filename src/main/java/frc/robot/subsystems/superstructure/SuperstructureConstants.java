package frc.robot.subsystems.superstructure;

import frc.robot.BuildConstants;

public class SuperstructureConstants {
    public static final double scoreCoralSettleSeconds = 0.5;
    public static final double scoreCoralL1SettleSeconds = 0.5;

    static SuperstructureIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL, REPLAY -> null;
            case SIM -> new SuperstructureIOSim();
        };
    }
}

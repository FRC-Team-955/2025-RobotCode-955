package frc.robot.subsystems.superstructure;

import frc.robot.BuildConstants;

public class SuperstructureConstants {
    static SuperstructureIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new SuperstructureIO();
            case SIM -> new SuperstructureIOSim();
            case REPLAY -> new SuperstructureIO();
        };
    }
}

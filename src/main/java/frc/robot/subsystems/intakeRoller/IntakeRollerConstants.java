package frc.robot.subsystems.intakeRoller;

import frc.lib.PIDF;
import frc.robot.BuildConstants;

public class IntakeRollerConstants {

    public static final IntakeRollerConfig intakeRollerConfig = switch (BuildConstants.mode) {
        case REAL, REPLAY -> null;
        case SIM -> new IntakeRollerConfig(
                false,
                true,
                40,
                3,
                PIDF.ofP(1),
                PIDF.ofPSVA(1, 0, 1, 1)

        );
    };

    static IntakeRollerIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL, REPLAY -> null;
            case SIM -> new IntakeRollerIOSim();
        };
    }


    public record IntakeRollerConfig(
            boolean inverted,
            boolean brakeMode,
            int currentLimit,
            double gearRatio,
            PIDF positionGains,
            PIDF velocityGains
    ) {
    }
}

package frc.robot.subsystems.intakerollers;

import frc.lib.PIDF;
import frc.robot.BuildConstants;
import frc.robot.subsystems.intakepivot.IntakePivotIO;
import frc.robot.subsystems.intakepivot.IntakePivotIOSim;

public class IntakeRollersConstants {
    public static final IntakeRollersConfig intakeRollersConfig = switch (BuildConstants.mode) {
        case REAL -> new IntakeRollersConfig(
                false,
                true,
                40,
                3,
//                PIDF.ofP(0),
                PIDF.ofPSVA(0, 0, 0, 0)
        );
        case SIM -> new IntakeRollersConfig(
                false,
                true,
                40,
                3,
//                PIDF.ofP(0.0),
                PIDF.ofPSVA(0.0, 0.002, 0.0569, 0.0)
        );
        case REPLAY -> null;
    };
    protected static final IntakeRollersIO intakeRollersIO = BuildConstants.mode == BuildConstants.Mode.REPLAY
            ? new IntakeRollersIO()
            : switch (BuildConstants.mode) {
        case REAL -> null; //new IntakeIOTalonFX();
        case SIM -> new IntakeRollersIOSim();
        case REPLAY -> new IntakeRollersIO();
    };

    public record IntakeRollersConfig(
            boolean inverted,
            boolean brakeMode,
            int currentLimit,
            double gearRatio,
//            PIDF positionGains,
            PIDF velocityGains
    ) {
    }
}


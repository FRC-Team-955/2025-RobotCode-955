package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.robot.BuildConstants;

public class IntakePivotConstants {
    public static final double intakeLengthMeters = Units.inchesToMeters(16);
    public static final double intakeSetpointToleranceRad = Units.degreesToRadians(15);
    public static final double intakeMaxVelocityRadPerSec = Units.degreesToRadians(500);
    public static final double intakeMaxAccelerationRadPerSecSquared = Units.degreesToRadians(1500);


    public static final IntakePivotConfig intakePivotConfig = switch (BuildConstants.mode) {
        case REAL, REPLAY -> null;
        case SIM -> new IntakePivotConfig(
                PIDF.ofPDSVAG(1.0, 0.0, 0.0, 15.0, 0.0, 1.4),
                60,
                false,
                false,
                40
        );
    };

    static IntakePivotIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL, REPLAY -> null;
            case SIM -> new IntakePivotIOSim();
        };
    }
//    protected static final IntakePivotIO createIO = BuildConstants.mode == BuildConstants.Mode.REPLAY
//            ? new IntakePivotIO()
//            : switch (BuildConstants.mode) {
//        case REAL, REPLAY -> null;
//        case SIM -> new IntakePivotIOSim();
//    };


    public record IntakePivotConfig(
            PIDF gains,
            double motorGearRatio,
            boolean motorInverted,
            boolean encoderInverted,
            double currentLimit
    ) {}

}


package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.robot.BuildConstants;

public class IntakeConstants {
    public static final double intakeLengthMeters = Units.inchesToMeters(16);
    public static final double intakeSetpointToleranceRad = Units.inchesToMeters(15);
    public static final double intakeMaxVelocityRadPerSec = Units.degreesToRadians(500);
    public static final double intakeMaxAccelerationRadPerSecSquared = Units.degreesToRadians(1500);

//    protected static RollersIO createRollersIO() {}
//    protected static final RollersIO rollersIo = BuildConstants.mode == BuildConstants.Mode.REPLAY
//            ? new RollersIO()
//            : switch (BuildConstants.mode) {
//        case REAL -> newRollersIOSparkMax();
//        case SIM -> new RollersIOSim(
//                new RollersConfig(
//                        false,
//                        true,
//                        40,
//                        3,
//                        PIDF.ofP(1),
//                        PIDF.ofPSVA(1, 0, 1, 1)
//                ),
//                0.01,
//                DCMotor.getNEO(1)
//        );
//    };

    public static final IntakeConfig intakeConfig = switch (BuildConstants.mode) {
        case REAL -> new IntakeConfig(
                PIDF.ofPIDSVAG(1, 0.0, 0.0, 0, 0, 0, 0),
                60,
                false,
                false,
                40
        );
        case SIM -> new IntakeConfig(
                PIDF.ofPDSVAG(1.0, 0.0, 0.0, 14.0, 0.0, 1.405),
                60,
                false,
                false,
                40
        );
        case REPLAY -> null;
    };

    // protected static IntakeIO createIntakeIO() {}
    protected static final IntakePivotIO intakePivotIO = BuildConstants.mode == BuildConstants.Mode.REPLAY
            ? new IntakePivotIO()
            : switch (BuildConstants.mode) {
        case REAL -> null; //new IntakeIOTalonFX();
        case SIM -> new IntakePivotIOSim();
        case REPLAY -> new IntakePivotIO();
    };

    public record IntakeConfig(
            PIDF gains,
            double motorGearRatio,
            boolean motorInverted,
            boolean encoderInverted,
            double currentLimit
    ) {}
}

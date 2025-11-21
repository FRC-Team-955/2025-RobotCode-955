package frc.robot.subsystems.drive.intake;

import edu.wpi.first.math.util.Units;
import frc.lib.PIDF;
import frc.robot.BuildConstants;

public class CoralIntakeConstants {
    public static final double pivotLengthMeters = Units.inchesToMeters(16);
    public static final double pivotSetpointToleranceRad = Units.degreesToRadians(15);
    public static final double pivotMaxVelocityRadPerSec = Units.degreesToRadians(500);
    public static final double pivotMaxAccelerationRadPerSecSquared = Units.degreesToRadians(1500);

   /* protected static RollersIO createRollersIO() {
    }

    protected static final RollersIO rollersIo = Constants.isReplay
            ? new RollersIO()
            : switch (Constants.identity) {
        case COMPBOT -> new RollersIOSparkMax();
        case SIMBOT -> new RollersIOSim(
                new RollersConfig(
                        false,
                        true,
                        40,
                        3,
                        PIDF.ofP(1),
                        PIDF.ofPSVA(1, 0, 1, 1)
                ),
                0.01,
                DCMotor.getNEO(1)
        );
        case ALPHABOT -> new RollersIO();
    }; */

    public static final PivotConfig pivotConfig = switch (BuildConstants.mode) {
        case REAL -> new PivotConfig(
                // TODO: Tune PID
                PIDF.ofPIDSVAG(1, 0.0, 0.0, 0, 0, 0, 0),
                60,
                false,
                // TODO: Figure this out
                false,
                40
        );
        case SIM -> new PivotConfig(
                PIDF.ofPSVAG(0.2, 0, 1.2, 1.2, 1.2),
                60,
                false,
                // TODO: Figure this out
                false,
                40
        );
        case REPLAY -> null;
    };

    /*protected static PivotIO createPivotIO() {}*/


    protected static final PivotIO pivotIo = BuildConstants.mode == BuildConstants.mode.REPLAY
            ? new PivotIO()
            : switch (BuildConstants.mode) {
        case REAL -> null;//new PivotIOTalonFX();
        case SIM -> new PivotIOSim();
        case REPLAY -> new PivotIO();
    };

    public record PivotConfig(
            PIDF gains,
            double motorGearRatio,
            boolean motorInverted,
            boolean encoderInverted,
            double currentLimit
    ) {
    }
}
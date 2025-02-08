package frc.robot.subsystems.coralintake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.util.PIDF;

public class CoralIntakeConstants {
    protected static final RollersIO rollersIo = Constants.isReplay
            ? new RollersIO()
            : switch (Constants.identity) {
        case COMPBOT -> null;
        case SIMBOT, ALPHABOT -> new RollersIOSim(
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
    };

    public static final PivotConfig pivotConfig = new PivotConfig(
            // TODO: Tune PID
            PIDF.ofPIDSVAG(1, 0.0, 0.0, 0, 0, 0, 0),
            60,
            false,
            // TODO: Figure this out
            false,
            0, // TODO
            40
    );


    protected static final PivotIO pivotIo = Constants.isReplay
            ? new PivotIO()
            : switch (Constants.identity) {
        case COMPBOT -> null;
        case SIMBOT, ALPHABOT -> new PivotIOSim(PIDF.ofPSVA(3, 0, 0, 1));
    };

    public record PivotConfig(
            PIDF motorGains,
            double motorGearRatio,
            boolean motorInverted,
            boolean encoderInverted,
            double setpointToleranceRad,
            double currentLimit
    ) {
    }
}

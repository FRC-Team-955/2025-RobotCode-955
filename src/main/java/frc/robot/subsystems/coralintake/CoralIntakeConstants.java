package frc.robot.subsystems.coralintake;

import frc.robot.util.PIDF;

public class CoralIntakeConstants {
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

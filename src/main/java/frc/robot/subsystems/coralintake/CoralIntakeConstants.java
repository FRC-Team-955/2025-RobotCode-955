package frc.robot.subsystems.coralintake;

import com.ctre.phoenix6.CANBus;
import frc.robot.Constants;
import frc.robot.util.PIDF;

public class CoralIntakeConstants {
    public static final String canbusName = "phoenix";

    public record PivotConfig(
            PIDF motorGains,
            double motorGearRatio,
            boolean motorInverted,
            boolean encoderInverted,
            int currentLimit
    ) {
    }

    public static final PivotConfig pivotConfig =
        new PivotConfig(
                // TODO: Tune PID
                PIDF.ofPIDSVAG(1, 0.0, 0.0, 0, 0, 0,0),
                25,
                false,
                // TODO: Figure this out
                false,
                60
        );
}

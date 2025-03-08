package frc.robot.subsystems.funnel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparkMax;
import frc.robot.util.PIDF;

public class FunnelConstants {
    public static final double rollersRadiusMeters = Units.inchesToMeters(2.25 / 2.0);

    // TODO: Tune this
    public static final RollersConfig beltConfig = new RollersConfig(
            false,
            true,
            40,
            25,
            PIDF.ofP(1),
            switch (Constants.identity) {
                case COMPBOT -> PIDF.ofPSV(0.01, 0.42461, 0.18272);
                case SIMBOT -> PIDF.ofSV(0.00995, 0.17859);
                case ALPHABOT -> PIDF.ofP(0);
            }
    );

    protected static RollersIO createBeltIO() {
        if (Constants.isReplay) {
            return new RollersIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new RollersIOSparkMax(8, beltConfig);
            case SIMBOT -> new RollersIOSim(
                    beltConfig,
                    0.01,
                    DCMotor.getNEO(1)
            );
            case ALPHABOT -> new RollersIO();
        };
    }
}

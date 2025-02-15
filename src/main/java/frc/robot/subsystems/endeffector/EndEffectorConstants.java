package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.util.PIDF;

public class EndEffectorConstants {
    public static final PIDF positionGains = PIDF.ofP(1);
    public static final PIDF velocityGains = PIDF.ofPSVA(1, 0, 1, 1);

    protected static final RollersIO rollersIO = Constants.isReplay
            ? new RollersIO()
            : switch (Constants.identity) {
        case COMPBOT -> new RollersIO();
        case SIMBOT, ALPHABOT -> new RollersIOSim(
                new RollersConfig(
                        false,
                        true,
                        40,
                        3,
                        positionGains,
                        velocityGains
                ),
                0.01,
                DCMotor.getNEO(1)
        );
    };
}
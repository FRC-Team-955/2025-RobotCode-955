package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparkMax;
import frc.robot.util.PIDF;

public class EndEffectorConstants {
    public static final PIDF positionGains = PIDF.ofP(1);
    public static final PIDF velocityGains = PIDF.ofPSV(0.01, 0.43576, 0.31280);

    protected static RollersIO createRollersIO() {
        if (Constants.isReplay) {
            return new RollersIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new RollersIOSparkMax(
                    7,
                    new RollersConfig(
                            false,
                            true,
                            40,
                            15,
                            positionGains,
                            velocityGains
                    )
            );
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
}
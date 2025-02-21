package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparkMax;
import frc.robot.util.PIDF;

public class EndEffectorConstants {
    public static final RollersConfig rollersConfig = new RollersConfig(
            false,
            true,
            40,
            9,
            PIDF.ofP(1),
            PIDF.ofPSV(0.01, 0.60171, 0.30060)
    );

    protected static RollersIO createRollersIO() {
        if (Constants.isReplay) {
            return new RollersIO();
        }
        return switch (Constants.identity) {
            case COMPBOT -> new RollersIOSparkMax(7, rollersConfig);
            case SIMBOT, ALPHABOT -> new RollersIOSim(
                    rollersConfig,
                    0.01,
                    DCMotor.getNEO(1)
            );
        };
    }
}
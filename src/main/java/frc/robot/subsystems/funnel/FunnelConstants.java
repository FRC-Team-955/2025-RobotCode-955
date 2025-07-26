package frc.robot.subsystems.funnel;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.PIDF;
import frc.robot.BuildConstants;
import frc.robot.subsystems.rollers.RollersConfig;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOSim;
import frc.robot.subsystems.rollers.RollersIOSparkMax;

public class FunnelConstants {
    public static final RollersConfig beltConfig = new RollersConfig(
            true,
            true,
            40,
            5,
            switch (BuildConstants.mode) {
                case REAL, REPLAY -> PIDF.ofP(0.2);
                case SIM -> PIDF.ofP(1.5);
            },
            switch (BuildConstants.mode) {
                case REAL, REPLAY -> PIDF.ofPSV(0.01, 0.21416, 0.10077);
                case SIM -> PIDF.ofSV(0.00995, 0.17859);
            }
    );

    protected static RollersIO createBeltIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new RollersIOSparkMax(5, beltConfig);
            case SIM -> new RollersIOSim(
                    beltConfig,
                    0.01,
                    DCMotor.getNEO(1)
            );
            case REPLAY -> new RollersIO();
        };
    }
}

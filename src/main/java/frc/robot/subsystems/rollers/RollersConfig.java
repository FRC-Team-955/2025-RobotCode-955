package frc.robot.subsystems.rollers;

import frc.robot.util.PIDF;

public record RollersConfig(
        boolean inverted,
        int currentLimit,
        double gearRatio,
        PIDF gains
) {
}

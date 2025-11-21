package frc.robot.subsystems.drive.rollers;

import frc.lib.PIDF;

public record RollersConfig(
        boolean inverted,
        boolean brakeMode,
        int currentLimit,
        double gearRatio,
        PIDF positionGains,
        PIDF velocityGains
) {
}

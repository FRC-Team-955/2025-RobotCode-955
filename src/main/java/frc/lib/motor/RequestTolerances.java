package frc.lib.motor;

import edu.wpi.first.math.util.Units;

public record RequestTolerances(
    double positionToleranceRad,
    double velocityToleranceRadPerSec,
    double velocityToleranceVolts
) {
    private static final double defaultPositionToleranceRad = Units.degreesToRadians(15);
    private static final double defaultVelocityToleranceRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(30);
    private static final double defaultVelocityToleranceVolts = 1.0;

    private static RequestTolerances defaults() {
        return new RequestTolerances(defaultPositionToleranceRad, defaultVelocityToleranceRadPerSec, defaultVelocityToleranceVolts);
    }

    private static RequestTolerances position(double positionToleranceRad) {
        return new RequestTolerances(positionToleranceRad, defaultVelocityToleranceRadPerSec, defaultVelocityToleranceVolts);
    }

    private static RequestTolerances velocity(double velocityToleranceRadPerSec) {
        return new RequestTolerances(defaultPositionToleranceRad, velocityToleranceRadPerSec, defaultVelocityToleranceVolts);
    }

    private static RequestTolerances voltage(double velocityToleranceVolts) {
        return new RequestTolerances(defaultPositionToleranceRad, defaultVelocityToleranceRadPerSec, velocityToleranceVolts);
    }
}

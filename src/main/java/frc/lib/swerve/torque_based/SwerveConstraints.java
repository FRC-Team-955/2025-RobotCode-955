package frc.lib.swerve.torque_based;

public record SwerveConstraints(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSecSquared,
        double maxAngularVelocityRadPerSec,
        double maxAngularAccelerationRadPerSecSquared
) {
}

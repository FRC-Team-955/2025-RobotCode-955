package frc.lib.swerve;

public record ModuleLimits (
        double maxDriveVelocityMetersPerSec,
        double maxDriveAccelerationMetersPerSecSquared,
        double maxTurnVelocityRadPerSec
) {
    public ModuleLimits times(double scalar) {
        return new ModuleLimits(
                maxDriveVelocityMetersPerSec * scalar,
                maxDriveAccelerationMetersPerSecSquared * scalar,
                maxTurnVelocityRadPerSec * scalar
        );
    }
}

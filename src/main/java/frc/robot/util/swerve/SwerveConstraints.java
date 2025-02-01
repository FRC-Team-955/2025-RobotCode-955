// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/path/PathConstraints.java

package frc.robot.util.swerve;


/**
 * Arbitrary constraints (in addition to maximum capabilities)
 *
 * @param maxVelocityMetersPerSec                Max linear velocity (M/S)
 * @param maxAccelerationMetersPerSecSquared     Max linear acceleration (M/S^2)
 * @param maxAngularVelocityRadPerSec            Max angular velocity (Rad/S)
 * @param maxAngularAccelerationRadPerSecSquared Max angular acceleration (Rad/S^2)
 */
public record SwerveConstraints(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSecSquared,
        double maxAngularVelocityRadPerSec,
        double maxAngularAccelerationRadPerSecSquared
) {
}

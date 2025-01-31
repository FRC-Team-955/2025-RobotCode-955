// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/path/PathConstraints.java

package frc.robot.util.swerve;


/**
 * Kinematic path following constraints
 *
 * @param maxVelocityMetersPerSec                Max linear velocity (M/S)
 * @param maxAccelerationMetersPerSecSquared     Max linear acceleration (M/S^2)
 * @param maxAngularVelocityRadPerSec            Max angular velocity (Rad/S)
 * @param maxAngularAccelerationRadPerSecSquared Max angular acceleration (Rad/S^2)
 * @param nominalVoltageVolts                    The nominal battery voltage (Volts)
 * @param unlimited                              Should the constraints be unlimited
 */
public record ModuleLimits(
        double maxVelocityMetersPerSec,
        double maxAccelerationMetersPerSecSquared,
        double maxAngularVelocityRadPerSec,
        double maxAngularAccelerationRadPerSecSquared,
        double nominalVoltageVolts,
        boolean unlimited) {
    /**
     * Kinematic path following constraints
     *
     * @param maxVelocityMPS                    Max linear velocity (M/S)
     * @param maxAccelerationMPSSq              Max linear acceleration (M/S^2)
     * @param maxAngularVelocityRadPerSec       Max angular velocity (Rad/S)
     * @param maxAngularAccelerationRadPerSecSq Max angular acceleration (Rad/S^2)
     * @param nominalVoltageVolts               The nominal battery voltage (Volts)
     */
    public ModuleLimits(
            double maxVelocityMPS,
            double maxAccelerationMPSSq,
            double maxAngularVelocityRadPerSec,
            double maxAngularAccelerationRadPerSecSq,
            double nominalVoltageVolts) {
        this(
                maxVelocityMPS,
                maxAccelerationMPSSq,
                maxAngularVelocityRadPerSec,
                maxAngularAccelerationRadPerSecSq,
                nominalVoltageVolts,
                false);
    }

    /**
     * Kinematic path following constraints
     *
     * @param maxVelocityMPS                    Max linear velocity (M/S)
     * @param maxAccelerationMPSSq              Max linear acceleration (M/S^2)
     * @param maxAngularVelocityRadPerSec       Max angular velocity (Rad/S)
     * @param maxAngularAccelerationRadPerSecSq Max angular acceleration (Rad/S^2)
     */
    public ModuleLimits(
            double maxVelocityMPS,
            double maxAccelerationMPSSq,
            double maxAngularVelocityRadPerSec,
            double maxAngularAccelerationRadPerSecSq) {
        this(
                maxVelocityMPS,
                maxAccelerationMPSSq,
                maxAngularVelocityRadPerSec,
                maxAngularAccelerationRadPerSecSq,
                12.0,
                false);
    }

    /**
     * Get unlimited ModuleLimits
     *
     * @param nominalVoltage The nominal battery voltage (Volts)
     * @return Unlimited constraints
     */
    public static ModuleLimits unlimitedConstraints(double nominalVoltage) {
        return new ModuleLimits(
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                nominalVoltage,
                true);
    }
}

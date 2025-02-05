// https://github.com/mjansen4857/pathplanner/blob/main/pathplannerlib/src/main/java/com/pathplanner/lib/util/DriveFeedforwards.java

package frc.robot.util.swerve.pathplanner;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

/**
 * Collection of different feedforward values for each drive module. If using swerve, these values
 * will all be in FL, FR, BL, BR order. If using a differential drive, these will be in L, R order.
 *
 * <p>NOTE: If using Choreo paths, all feedforwards but the X and Y component arrays will be filled
 * with zeros.
 *
 * @param accelerationsMetersPerSecSquared Linear acceleration at the wheels in meters per second
 * @param linearForcesNewtons              Linear force applied by the motors at the wheels in newtons
 * @param torqueCurrentsAmps               Torque-current of the drive motors in amps
 * @param robotRelativeForcesXNewtons      X components of robot-relative force vectors for the wheels in
 *                                         newtons. The magnitude of these vectors will typically be greater than the linear force
 *                                         feedforwards due to friction forces.
 * @param robotRelativeForcesYNewtons      X components of robot-relative force vectors for the wheels in
 *                                         newtons. The magnitude of these vectors will typically be greater than the linear force
 *                                         feedforwards due to friction forces.
 */
public record DriveFeedforwards(
        double[] accelerationsMetersPerSecSquared,
        double[] linearForcesNewtons,
        double[] torqueCurrentsAmps,
        double[] robotRelativeForcesXNewtons,
        double[] robotRelativeForcesYNewtons)
        implements Interpolatable<DriveFeedforwards> {
    /**
     * Create drive feedforwards consisting of all zeros
     *
     * @param numModules Number of drive modules
     *
     * @return Zero feedforwards
     */
    public static DriveFeedforwards zeros(int numModules) {
        return new DriveFeedforwards(
                new double[numModules],
                new double[numModules],
                new double[numModules],
                new double[numModules],
                new double[numModules]);
    }

    @Override
    public DriveFeedforwards interpolate(DriveFeedforwards endValue, double t) {
        return new DriveFeedforwards(
                interpolateArray(accelerationsMetersPerSecSquared, endValue.accelerationsMetersPerSecSquared, t),
                interpolateArray(linearForcesNewtons, endValue.linearForcesNewtons, t),
                interpolateArray(torqueCurrentsAmps, endValue.torqueCurrentsAmps, t),
                interpolateArray(robotRelativeForcesXNewtons, endValue.robotRelativeForcesXNewtons, t),
                interpolateArray(robotRelativeForcesYNewtons, endValue.robotRelativeForcesYNewtons, t));
    }

    /**
     * Reverse the feedforwards for driving backwards. This should only be used for differential drive
     * robots.
     *
     * @return Reversed feedforwards
     */
    public DriveFeedforwards reverse() {
        if (accelerationsMetersPerSecSquared.length != 2) {
            throw new IllegalStateException(
                    "Feedforwards should only be reversed for differential drive trains");
        }

        return new DriveFeedforwards(
                new double[]{-accelerationsMetersPerSecSquared[1], -accelerationsMetersPerSecSquared[0]},
                new double[]{-linearForcesNewtons[1], -linearForcesNewtons[0]},
                new double[]{-torqueCurrentsAmps[1], -torqueCurrentsAmps[0]},
                new double[]{-robotRelativeForcesXNewtons[1], -robotRelativeForcesXNewtons[0]},
                new double[]{-robotRelativeForcesYNewtons[1], -robotRelativeForcesYNewtons[0]});
    }

    /**
     * Flip the feedforwards for the other side of the field. Only does anything if mirrored symmetry
     * is used
     *
     * @return Flipped feedforwards
     */
    public DriveFeedforwards flip() {
        return new DriveFeedforwards(
                FlippingUtil.flipFeedforwards(accelerationsMetersPerSecSquared),
                FlippingUtil.flipFeedforwards(linearForcesNewtons),
                FlippingUtil.flipFeedforwards(torqueCurrentsAmps),
                FlippingUtil.flipFeedforwardXs(robotRelativeForcesXNewtons),
                FlippingUtil.flipFeedforwardYs(robotRelativeForcesYNewtons));
    }

    private static double[] interpolateArray(double[] a, double[] b, double t) {
        double[] ret = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            ret[i] = MathUtil.interpolate(a[i], b[i], t);
        }
        return ret;
    }
}
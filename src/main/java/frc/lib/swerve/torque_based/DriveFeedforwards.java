package frc.lib.swerve.torque_based;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public record DriveFeedforwards(
        double[] accelerationsMetersPerSecSquared,
        double[] linearForcesNewtons,
        double[] torqueCurrentsAmps,
        double[] robotRelativeForcesXNewtons,
        double[] robotRelativeForcesYNewtons)
        implements Interpolatable<DriveFeedforwards> {

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
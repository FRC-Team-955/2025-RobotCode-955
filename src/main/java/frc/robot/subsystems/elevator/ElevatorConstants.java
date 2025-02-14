package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PIDF;

public class ElevatorConstants {
    public static final boolean useCurrentControl = true;

    /** Gains in radians */
    public static final PIDF gains = switch (Constants.identity) {
        case COMPBOT -> PIDF.ofPDSVAG(0, 0, 0.1, 0.08, 0, 0.5);
        case SIMBOT, ALPHABOT -> useCurrentControl
                ? PIDF.ofPDSG(30, 0.0, 0, 2.50)
                : PIDF.ofPDSVAG(0.1, 0.0, 0, 0.06, 0.018, 2.9296);
    };

    public static final double maxVelocityMetersPerSecond = 5;
    public static final double maxAccelerationMetersPerSecondSquared = 10;

    public static final double gearRatio = 3;
    private static final double sprocketRadiusMeters = Units.inchesToMeters((1.0 + (9.0 / 32.0)) / 2);
    public static final double drumRadiusMeters = sprocketRadiusMeters * 3; // 3 stages
    public static final double maxHeightMeters = Units.inchesToMeters(66.622);

    public static final double hardstopMeters = Units.inchesToMeters(10.66);
    public static final double gentleMaxVelocityMetersPerSecond = maxVelocityMetersPerSecond / 4;
    /**
     * Not actually the max acceleration, just the max acceleration we assume when calculating slowdown
     * height in order to be a little conservative.
     */
    private static final double assumedMaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared * 0.95;
    /**
     * While we could calculate this based on the current velocity, it caused the gentle profile to be used
     * for only half of the loop cycles. This could probably be solved but I don't think it's worth the effort
     */
    public static final double hardstopSlowdownMeters = calculateHardstopSlowdownMeters(maxVelocityMetersPerSecond);

    private static double calculateHardstopSlowdownMeters(double currentVelocityMetersPerSec) {
        // If we are going down at v:
        //     x = -v * t
        // If we slow down at a, max acceleration,
        //     x = -v * t + (1/2) * a * t^2
        // Relating v to the max velocity we want before hitting the hardstop, v_g:
        //     -v_g = -v + a * t
        //     (v - v_g) / a = t
        // Substituting t and doing some simplification:
        //     x = (-v * (v - v_g)) / a + ((v - v_g)^2) / (2 * a)
        // Separating the components to make more readable:
        //     d = (v - v_g)
        double d = currentVelocityMetersPerSec - gentleMaxVelocityMetersPerSecond;
        //     x_v = (-v * d) / a
        double x_v = (-currentVelocityMetersPerSec * d) / assumedMaxAccelerationMetersPerSecondSquared;
        //     x_a = d^2 / (2 * a)
        double x_a = d * d / (2 * assumedMaxAccelerationMetersPerSecondSquared);
        //     x = x_v + x_a
        double x = x_v + x_a;
        // x is negative because we're going down, we want a positive distance above hardstop
        return hardstopMeters + -x;
    }

    public static final double setpointToleranceRad = metersToRad(Units.inchesToMeters(2));

    public static double metersToRad(double meters) {
        return meters / drumRadiusMeters;
    }

    public static double radToMeters(double rad) {
        return rad * drumRadiusMeters;
    }

    // IO layers should go at the bottom in case they reference constants that aren't yet initialized
    // TODO: function to avoid this?

    protected static final ElevatorIO io = Constants.isReplay
            ? new ElevatorIO()
            : switch (Constants.identity) {
        case COMPBOT -> new ElevatorIO();
//        case COMPBOT -> new ElevatorIOSparkMax(5, 6, 9, true);
        case SIMBOT, ALPHABOT -> new ElevatorIOSim();
    };
}
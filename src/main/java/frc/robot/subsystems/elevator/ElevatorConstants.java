package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PIDF;

public class ElevatorConstants {
    /** Gains in radians */
    public static final PIDF gains = switch (Constants.identity) {
        case COMPBOT -> PIDF.ofPDSVAG(0.03, 0, 0, 0.1, 0.01, 0.35);
        case SIMBOT, ALPHABOT -> PIDF.ofPDVAG(0, 0, 0.1, 0.008, 1.5015);
    };

    public static final double maxVelocityMetersPerSecond = 0.3;
    public static final double maxAccelerationMetersPerSecondSquared = 0.3;

    public static final double gearRatio = 5;
    private static final double sprocketRadiusMeters = Units.inchesToMeters((1.0 + (9.0 / 32.0)) / 2);
    public static final double drumRadiusMeters = sprocketRadiusMeters * 3; // 3 stages

    public static final double setpointToleranceMeters = Units.inchesToMeters(5);

    public static final double maxHeightMeters = Units.inchesToMeters(66.622);
    public static final ElevatorLimit upperLimit = new ElevatorLimit(maxHeightMeters - 0.25, 2);
    public static final ElevatorLimit lowerLimit = new ElevatorLimit(0.25, -1.5);

    public static final double hardstopMeters = Units.inchesToMeters(10.66);
    public static final double gentleMaxVelocityMetersPerSecond = 2;
    /**
     * Not actually the max acceleration, just the max acceleration we assume when calculating slowdown
     * height in order to be a little conservative.
     */
    private static final double assumedMaxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared * 0.95;
    /**
     * While we could calculate this based on the current velocity, it caused the gentle profile to be used
     * for only half of the loop cycles. This could probably be solved but I don't think it's worth the effort
     */
    public static double hardstopSlowdownMeters = calculateHardstopSlowdownMeters(maxVelocityMetersPerSecond);

    public static double calculateHardstopSlowdownMeters(double currentVelocityMetersPerSec) {
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
        case COMPBOT -> new ElevatorIOSparkMax(5, 6, 9, true);
        case SIMBOT, ALPHABOT -> new ElevatorIOSim();
    };

    public record ElevatorLimit(
            double positionMeters,
            double velocityMetersPerSec
    ) {
    }
}
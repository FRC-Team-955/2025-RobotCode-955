package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PIDF;

public class ElevatorConstants {
    /** Gains in radians */
    public static final PIDF gains = switch (Constants.identity) {
        case COMPBOT -> null;
        case SIMBOT, ALPHABOT -> PIDF.ofPDSVAG(0.1, 0.0, 0, 0.06, 0.018, 2.9296);
    };

    public static final double maxVelocityMetersPerSecond = 2;
    public static final double maxAccelerationMetersPerSecondSquared = 10;

    public static final double gearRatio = 3;
    private static final double sprocketRadiusMeters = Units.inchesToMeters(0.75);
    public static final double drumRadiusMeters = sprocketRadiusMeters * 3; // 3 stages

    //    public static final double hardstopHeightMeters = 1;
    public static final double gentleMaxVelocityMetersPerSecond = maxVelocityMetersPerSecond / 2;
//    // will explain this later
//    private static final double hardstopSlowdownSeconds = (maxVelocitySlowdownZoneMetersPerSecond - maxVelocityMetersPerSecond) / maxAccelerationMetersPerSecondSquared;
//    public static final double hardstopSlowdownMeters = hardstopHeightMeters + ;
    // TODO: should only slowdown on the way down; should not speed up after past hardstop on the way down

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
        case COMPBOT -> null;
        case SIMBOT, ALPHABOT -> new ElevatorIOSim();
    };
}
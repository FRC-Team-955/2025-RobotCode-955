package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    protected static final ElevatorIO io = null;

    public static final double gearRatio = 1;
    private static final double radPerMeter = 1;
    private static final double metersPerRad = 1 / radPerMeter;

    public static final double hardStopHeightMeters = 1;
    public static final double maxVelocityAboveHardStopMetersPerSecond = 1;
    public static final double maxVelocityBelowHardStopMetersPerSecond = 1;

    public static final double setpointToleranceRad = metersToRad(Units.inchesToMeters(3));

    public static double metersToRad(double meters) {
        return meters * radPerMeter;
    }

    public static double radToMeters(double rad) {
        return rad * metersPerRad;
    }
}
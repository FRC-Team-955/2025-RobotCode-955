package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

public final class ShooterRegression {
    /*
    distance to speaker (m) | shooter angle (degrees) | shooter speed (RPM)
    ------------------------|-------------------------|--------------------
    1.337                   | -50                     | 3500
    2.390                   | -34                     | 4000
    3.3                     | -28.5                   | 4250
    4.05                    | -25.5                   | 5000
    4.342                   | -22.3                   | 5200
    */

    public static Measure<Angle> getAngle(Measure<Distance> distance) {
        var distanceMeters = distance.in(Meters);
        var angleRad = Math.atan(-1.23351 / (distanceMeters - 0.20496)) - 0.0332181 * distanceMeters;
        var clampedAngleRad = MathUtil.clamp(angleRad, Units.degreesToRadians(-60), 0);
        return Radians.of(clampedAngleRad);
    }

    public static Measure<Velocity<Angle>> getSpeed(Measure<Distance> distance) {
        var distanceMeters = distance.in(Meters);
        var speedRPM = Math.min(51.0189 * Math.pow(distanceMeters, 2) + 276.094 * distanceMeters + 3041.66, 6000);
        var clampedSpeedRPM = MathUtil.clamp(speedRPM, 1000, 6000);
        return RPM.of(clampedSpeedRPM);
    }
}

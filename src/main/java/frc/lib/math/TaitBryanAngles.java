package frc.lib.math;

import edu.wpi.first.math.geometry.Quaternion;

/**
 * Rotation order is yaw-pitch-roll. See https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles
 */
public class TaitBryanAngles {
    public final double yawRad;
    public final double pitchRad;
    public final double rollRad;

    public TaitBryanAngles(Quaternion q) {
        // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
        yawRad = Math.atan2(2.0 * (q.getW() * q.getZ() + q.getX() * q.getY()), 1.0 - 2.0 * (q.getY() * q.getY() + q.getZ() * q.getZ()));
        pitchRad = (-Math.PI / 2.0) + 2.0 * Math.atan2(Math.sqrt(1 + 2.0 * (q.getW() * q.getY() - q.getX() * q.getZ())), Math.sqrt(1 - 2.0 * (q.getW() * q.getY() - q.getX() * q.getZ())));
        rollRad = Math.atan2(2.0 * (q.getW() * q.getX() + q.getY() * q.getZ()), 1.0 - 2.0 * (q.getX() * q.getX() + q.getY() * q.getY()));
    }
}

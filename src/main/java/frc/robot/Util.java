package frc.robot;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.EnumMap;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.function.Function;

public class Util {
    private static final double epsilon = 1E-6;

    public static final Executor asyncExecutor = Executors.newFixedThreadPool(4);

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static Rotation2d flipIfNeeded(Rotation2d rotation2d) {
        return shouldFlip()
                ? rotation2d.plus(Rotation2d.kPi)
                : rotation2d;
    }

    public static Pose2d flipIfNeeded(Pose2d pose2d) {
        return shouldFlip()
                ? ChoreoAllianceFlipUtil.flip(pose2d)
                : pose2d;
    }

    public static void error(String msg) {
        if (BuildConstants.mode == BuildConstants.Mode.SIM) {
            throw new RuntimeException(msg);
        } else {
            DriverStation.reportError(msg, false);
        }
    }

    public static boolean epsilonEquals(double a, double b) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
        return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
                && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
                && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
    }

    public static boolean greaterThanEpsilon(double a) {
        return a > epsilon;
    }

    public static int positiveModulus(int input, int modulus) {
        // Ensures returned value is positive
        return ((input % modulus) + modulus) % modulus;
    }

    public static <E extends Enum<E>, V> EnumMap<E, V> createEnumMap(Class<E> clazz, E[] values, Function<E, V> valueSupplier) {
        EnumMap<E, V> map = new EnumMap<>(clazz);
        for (E key : values) {
            map.put(key, valueSupplier.apply(key));
        }
        return map;
    }

    public static boolean isAtPoseWithTolerance(Pose2d currentPose, Pose2d desiredPose, double linearToleranceMeters, double angularToleranceRad) {
        Transform2d relative = new Transform2d(desiredPose, currentPose);
        return Math.abs(relative.getTranslation().getNorm()) < linearToleranceMeters
                && Math.abs(MathUtil.angleModulus(relative.getRotation().getRadians())) < angularToleranceRad;
    }

    public static boolean isWithinVelocityTolerance(ChassisSpeeds measuredChassisSpeeds, double linearToleranceMetersPerSec, double angularToleranceRadPerSec) {
        return Math.hypot(measuredChassisSpeeds.vxMetersPerSecond, measuredChassisSpeeds.vyMetersPerSecond) < linearToleranceMetersPerSec
                && Math.abs(measuredChassisSpeeds.omegaRadiansPerSecond) < angularToleranceRadPerSec;
    }
}
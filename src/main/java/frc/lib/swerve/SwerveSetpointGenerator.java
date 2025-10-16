package frc.lib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.PhoenixUtil;
import frc.lib.Util;
import lombok.experimental.ExtensionMethod;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


@ExtensionMethod({Util.class})
public class SwerveSetpointGenerator {
    private static final boolean useIterative = false;
    private static final int maxSteeringIterations = 8;
    private static final int maxDriveIterations = 10;

    private final SwerveDriveKinematics kinematics;
    private final int numberOfModules;

    public SwerveSetpointGenerator(SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
        numberOfModules = kinematics.getModules().length;
    }

    public SwerveSetpoint generateSetpoint(
            final ModuleLimits limits,
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredState,
            double dt
    ) {
        SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredState);
        if (limits.maxDriveVelocityMetersPerSec() > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.maxDriveVelocityMetersPerSec());
            desiredState = kinematics.toChassisSpeeds(desiredModuleState);
        }

        boolean need_to_steer = true;
        if (desiredState.epsilonEquals(new ChassisSpeeds())) {
            need_to_steer = false;
            for (int i = 0; i < numberOfModules; ++i) {
                desiredModuleState[i].angle = prevSetpoint.moduleStates()[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        double[] prev_vx = new double[numberOfModules];
        double[] prev_vy = new double[numberOfModules];
        Rotation2d[] prev_heading = new Rotation2d[numberOfModules];
        double[] desired_vx = new double[numberOfModules];
        double[] desired_vy = new double[numberOfModules];
        Rotation2d[] desired_heading = new Rotation2d[numberOfModules];
        boolean all_modules_should_flip = true;
        for (int i = 0; i < numberOfModules; ++i) {
            prev_vx[i] = prevSetpoint.moduleStates()[i].angle.getCos() * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
            prev_vy[i] = prevSetpoint.moduleStates()[i].angle.getSin() * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
            prev_heading[i] = prevSetpoint.moduleStates()[i].angle;
            if (prevSetpoint.moduleStates()[i].speedMetersPerSecond < 0.0) {
                prev_heading[i] = prev_heading[i].rotateBy(Rotation2d.k180deg);
            }
            desired_vx[i] = desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
            desired_vy[i] = desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
            desired_heading[i] = desiredModuleState[i].angle;
            if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
                desired_heading[i] = desired_heading[i].rotateBy(Rotation2d.k180deg);
            }
            if (all_modules_should_flip) {
                double required_rotation_rad = Math.abs(
                        prev_heading[i]
                                .unaryMinus()
                                .rotateBy(desired_heading[i])
                                .getRadians()
                );
                if (required_rotation_rad < Math.PI / 2.0) {
                    all_modules_should_flip = false;
                }
            }
        }
        if (all_modules_should_flip
                && !prevSetpoint.chassisSpeeds().epsilonEquals(new ChassisSpeeds())
                && !desiredState.epsilonEquals(new ChassisSpeeds())
        ) {
            return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
        }

        double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

        double min_s = 1.0;

        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(numberOfModules);

        final double max_theta_step = dt * limits.maxTurnVelocityRadPerSec();
        for (int i = 0; i < numberOfModules; ++i) {
            if (need_to_steer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());
            if (epsilonEquals(prevSetpoint.moduleStates()[i].speedMetersPerSecond, 0.0)) {
                if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates()[i].angle));
                }

                var necessaryRotation = prevSetpoint.moduleStates()[i].angle
                        .unaryMinus()
                        .rotateBy(desiredModuleState[i].angle);
                if (shouldFlipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.k180deg);
                }

                final double numStepsNeeded = Math.abs(MathUtil.angleModulus(necessaryRotation.getRadians())) / max_theta_step;

                if (numStepsNeeded <= 1.0) {
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    continue;
                } else {
                    overrideSteering.set(
                            i,
                            Optional.of(
                                    prevSetpoint.moduleStates()[i].angle.rotateBy(
                                            Rotation2d.fromRadians(
                                                    Math.signum(necessaryRotation.getRadians()) * max_theta_step
                                            )
                                    )
                            )
                    );
                    min_s = 0.0;
                    continue;
                }
            }
            if (min_s == 0.0) {
                continue;
            }

            double s;
            if (useIterative) {
                s = findSteeringMaxSIterative(
                        prev_vx[i],
                        prev_vy[i],
                        prev_heading[i].getRadians(),
                        desired_vx[i],
                        desired_vy[i],
                        desired_heading[i].getRadians(),
                        max_theta_step,
                        maxSteeringIterations
                );
            } else {
                s = findSteeringMaxSDirect(
                        prev_vx[i],
                        prev_vy[i],
                        prev_heading[i].getRadians(),
                        desired_vx[i],
                        desired_vy[i],
                        desired_heading[i].getRadians(),
                        max_theta_step
                );
            }
            min_s = Math.min(min_s, s);
        }

        final double max_vel_step = dt * limits.maxDriveAccelerationMetersPerSecSquared();
        for (int i = 0; i < numberOfModules; ++i) {
            if (min_s == 0.0) {
                break;
            }
            double vx_min_s = min_s == 1.0 ? desired_vx[i] : (desired_vx[i] - prev_vx[i]) * min_s + prev_vx[i];
            double vy_min_s = min_s == 1.0 ? desired_vy[i] : (desired_vy[i] - prev_vy[i]) * min_s + prev_vy[i];
            double s;
            if (useIterative) {
                s = min_s * findDriveMaxSIterative(
                        prev_vx[i],
                        prev_vy[i],
                        Math.hypot(prev_vx[i], prev_vy[i]),
                        vx_min_s,
                        vy_min_s,
                        Math.hypot(vx_min_s, vy_min_s),
                        max_vel_step,
                        maxDriveIterations
                );
            } else {
                s = min_s * findDriveMaxSDirect(
                        prev_vx[i],
                        prev_vy[i],
                        vx_min_s,
                        vy_min_s,
                        max_vel_step
                );
            }
            min_s = Math.min(min_s, s);
        }
        ChassisSpeeds retSpeeds = new ChassisSpeeds(
                prevSetpoint.chassisSpeeds().vxMetersPerSecond + min_s * dx,
                prevSetpoint.chassisSpeeds().vyMetersPerSecond + min_s * dy,
                prevSetpoint.chassisSpeeds().omegaRadiansPerSecond + min_s * dtheta
        );
        retSpeeds = ChassisSpeeds.discretize(retSpeeds, dt);
        var retStates = kinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < numberOfModules; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (shouldFlipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
                    retStates[i].speedMetersPerSecond *= -1.0;
                }
                retStates[i].angle = override;
            }
            final var deltaRotation = prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(retStates[i].angle);
            if (shouldFlipHeading(deltaRotation)) {
                retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.k180deg);
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoint(retSpeeds, retStates);
    }
    private static double findSteeringMaxSDirect (
            double x_0,
            double y_0,
            double theta_0,
            double x_1,
            double y_1,
            double theta_1,
            double max_deviation
    ) {
        theta_1 = unwrapAngle(theta_0, theta_1);
        double diff = theta_1 - theta_0;
        if (Math.abs(diff) <= max_deviation) {
            return 1.0;
        }

        double target = theta_0 + Math.copySign(max_deviation, diff);
        double sin = Math.sin(-target);
        double cos = Math.cos(-target);
        double h_0 = sin * x_0 + cos * y_0;
        double h_1 = sin * x_1 + cos * y_1;

        return h_0 / (h_0 - h_1);
    }

    private static boolean isValidS(double s) {
        return Double.isFinite(s)
                && s >= 0
                && s <= 1;
    }

    private static double findDriveMaxSDirect(
            double x_0,
            double y_0,
            double x_1,
            double y_1,
            double max_vel_step
    ) {
        double l_0 = x_0 * x_0 + y_0 * y_0;
        double l_1 = x_1 * x_1 + y_1 * y_1;
        double sqrt_l_0 = Math.sqrt(l_0);
        double sqrt_l_1 = Math.sqrt(l_1);
        double diff = sqrt_l_1 - sqrt_l_0;
        if (Math.abs(diff) <= max_vel_step) {
            return 1.0;
        }

        double target = sqrt_l_0 + Math.copySign(max_vel_step, diff);
        double p = x_0 * x_1 + y_0 * y_1;

        double a = l_0 + l_1 - 2 * p;
        double b = 2 * (p - l_0);
        double c = l_0 - target * target;
        double root = Math.sqrt(b * b - 4 * a * c);

        double s_1 = (-b + root) / (2 * a);
        if (isValidS(s_1)) {
            return s_1;
        }
        double s_2 = (-b - root) / (2 * a);
        if (isValidS(s_2)) {
            return s_2;
        }
        return 1.0;
    }

    private static double findRoot(
            Function2d func,
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            int iterations_left
    ) {
        if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
            return 1.0;
        }
        var s_guess = Math.max(0.0, Math.min(-1.0, -f_0 / (f_1 - f_0)));
        var x_guess = (x_1 - x_0) * s_guess + x_0;
        var y_guess = (y_1 - y_0) * s_guess + y_0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f_0) == Math.signum(f_guess)) {
            return s_guess
                    + (1.0 + s_guess)
                    * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
        } else {
            return s_guess
                    * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    protected static double findSteeringMaxSIterative(
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_deviation,
            int max_iterations
    ) {
        f_1 = unwrapAngle(f_0, f_1);
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_deviation) {
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_deviation;
        Function2d func = (x, y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    protected static double findDriveMaxSIterative (
            double x_0,
            double y_0,
            double f_0,
            double x_1,
            double y_1,
            double f_1,
            double max_vel_step,
            int max_iterations
    ) {
        double diff = f_1 - f_0;
        if (Math.abs(diff) <= max_vel_step) {
            return 1.0;
        }
        double offset = f_0 + Math.signum(diff) * max_vel_step;
        Function2d func = (x, y) -> Math.hypot(x, y) - offset;
        return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
    }

    private static boolean shouldFlipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    @FunctionalInterface
    private interface Function2d {
        double f(double x, double y);
    }
}

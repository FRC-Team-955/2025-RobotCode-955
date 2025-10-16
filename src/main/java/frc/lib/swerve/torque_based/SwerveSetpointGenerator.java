package frc.lib.swerve.torque_based;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import static frc.lib.Util.epsilonEquals;

public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics kinematics;
    private final SimpleMatrix forceKinematics;
    private final Translation2d[] moduleLocations;
    private final double[] moduleDistances;

    private final double maxTurnVelocityRadPerSec;
    private final double maxDriveVelocityMetersPerSec;
    private final double wheelRadiusMeters;
    private final double wheelFrictionNewtons;
    private final double massKG;
    public final double MOIKGMetersSquared;

    private final DCMotor driveMotor;
    private final double driveCurrentLimit;
    private final double driveTorqueLoss;
    public final double driveMaxTorqueWithoutSlip;

    private final double brownoutVoltage;

    public SwerveSetpointGenerator(
            SwerveDriveKinematics kinematics,
            double maxTurnVelocityRadPerSec,
            double maxDriveVelocityMetersPerSec,
            double wheelRadiusMeters,
            double wheelCOF,
            double massKG,
            double MOIKGMetersSquared,
            DCMotor driveMotor,
            double driveCurrentLimit
    ) {
        this.kinematics = kinematics;
        moduleLocations = kinematics.getModules();
        moduleDistances = Arrays.stream(moduleLocations).mapToDouble(Translation2d::getNorm).toArray();
        forceKinematics = new SimpleMatrix(moduleLocations.length * 2, 3);
        for (int i = 0; i < moduleLocations.length; i++) {
            Translation2d modPosReciprocal = new Translation2d(
                    1.0 / moduleLocations[i].getNorm(),
                    moduleLocations[i].getAngle()
            );
            forceKinematics.setRow(i * 2, 0, 1, 0, -modPosReciprocal.getY());
            forceKinematics.setRow(i * 2 + 1, 0, 0, 1, modPosReciprocal.getX());
        }

        this.maxTurnVelocityRadPerSec = maxTurnVelocityRadPerSec;
        this.maxDriveVelocityMetersPerSec = maxDriveVelocityMetersPerSec;
        this.wheelRadiusMeters = wheelRadiusMeters;
        this.massKG = massKG;
        this.MOIKGMetersSquared = MOIKGMetersSquared;
        this.driveMotor = driveMotor;
        this.driveCurrentLimit = driveCurrentLimit;
        this.wheelFrictionNewtons = wheelCOF * ((massKG / moduleLocations.length) * 9.8);
        this.driveMaxTorqueWithoutSlip = wheelFrictionNewtons * wheelRadiusMeters;
        double maxDriveVelocityRadPerSec = maxDriveVelocityMetersPerSec / this.wheelRadiusMeters;
        double maxSpeedCurrentDraw = this.driveMotor.getCurrent(maxDriveVelocityRadPerSec, 12.0);
        this.driveTorqueLoss = Math.max(
                this.driveMotor.getTorque(Math.min(maxSpeedCurrentDraw, this.driveCurrentLimit)),
                0.0
        );

        this.brownoutVoltage = RobotController.getBrownoutVoltage();
    }

    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredStateRobotRelative,
            final SwerveConstraints constraints,
            double dt,
            double inputVoltage) {
        if (Double.isNaN(inputVoltage)) {
            inputVoltage = 12.0;
        } else {
            inputVoltage = Math.max(inputVoltage, brownoutVoltage);
        }
        double maxSpeed = maxDriveVelocityMetersPerSec * Math.min(1, inputVoltage / 12);

        if (constraints != null) {
            Translation2d vel = new Translation2d(
                    desiredStateRobotRelative.vxMetersPerSecond,
                    desiredStateRobotRelative.vyMetersPerSecond
            );
            double linearVel = vel.getNorm();
            if (linearVel > constraints.maxVelocityMetersPerSec()) {
                vel = vel.times(constraints.maxVelocityMetersPerSec() / linearVel);
            }
            desiredStateRobotRelative = new ChassisSpeeds(
                    vel.getX(),
                    vel.getY(),
                    MathUtil.clamp(
                            desiredStateRobotRelative.omegaRadiansPerSecond,
                            -constraints.maxAngularVelocityRadPerSec(),
                            constraints.maxAngularVelocityRadPerSec()
                    )
            );
        }

        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredStateRobotRelative);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, maxSpeed);
        desiredStateRobotRelative = kinematics.toChassisSpeeds(desiredModuleStates);

        boolean need_to_steer = true;
        if (epsilonEquals(desiredStateRobotRelative, new ChassisSpeeds())) {
            need_to_steer = false;
            for (int m = 0; m < moduleLocations.length; m++) {
                desiredModuleStates[m].angle = prevSetpoint.moduleStates()[m].angle;
                desiredModuleStates[m].speedMetersPerSecond = 0.0;
            }
        }

        double[] prev_vx = new double[moduleLocations.length];
        double[] prev_vy = new double[moduleLocations.length];
        Rotation2d[] prev_heading = new Rotation2d[moduleLocations.length];
        double[] desired_vx = new double[moduleLocations.length];
        double[] desired_vy = new double[moduleLocations.length];
        Rotation2d[] desired_heading = new Rotation2d[moduleLocations.length];
        boolean all_modules_should_flip = true;
        for (int m = 0; m < moduleLocations.length; m++) {
            prev_vx[m] =
                    prevSetpoint.moduleStates()[m].angle.getCos()
                            * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            prev_vy[m] =
                    prevSetpoint.moduleStates()[m].angle.getSin()
                            * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            prev_heading[m] = prevSetpoint.moduleStates()[m].angle;
            if (prevSetpoint.moduleStates()[m].speedMetersPerSecond < 0.0) {
                prev_heading[m] = prev_heading[m].rotateBy(Rotation2d.k180deg);
            }
            desired_vx[m] =
                    desiredModuleStates[m].angle.getCos() * desiredModuleStates[m].speedMetersPerSecond;
            desired_vy[m] =
                    desiredModuleStates[m].angle.getSin() * desiredModuleStates[m].speedMetersPerSecond;
            desired_heading[m] = desiredModuleStates[m].angle;
            if (desiredModuleStates[m].speedMetersPerSecond < 0.0) {
                desired_heading[m] = desired_heading[m].rotateBy(Rotation2d.k180deg);
            }
            if (all_modules_should_flip) {
                double required_rotation_rad =
                        Math.abs(prev_heading[m].unaryMinus().rotateBy(desired_heading[m]).getRadians());
                if (required_rotation_rad < Math.PI / 2.0) {
                    all_modules_should_flip = false;
                }
            }
        }
        if (all_modules_should_flip
                && !epsilonEquals(prevSetpoint.robotRelativeSpeeds(), new ChassisSpeeds())
                && !epsilonEquals(desiredStateRobotRelative, new ChassisSpeeds())) {
            return generateSetpoint(prevSetpoint, new ChassisSpeeds(), constraints, dt, inputVoltage);
        }

        double dx = desiredStateRobotRelative.vxMetersPerSecond
                - prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond;
        double dy = desiredStateRobotRelative.vyMetersPerSecond
                - prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond;
        double dtheta = desiredStateRobotRelative.omegaRadiansPerSecond
                - prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond;
        double min_s = 1.0;

        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(moduleLocations.length);
        for (int m = 0; m < moduleLocations.length; m++) {
            if (!need_to_steer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[m].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());

            double max_theta_step = dt * maxTurnVelocityRadPerSec;

            if (epsilonEquals(prevSetpoint.moduleStates()[m].speedMetersPerSecond, 0.0)) {
                if (epsilonEquals(desiredModuleStates[m].speedMetersPerSecond, 0.0)) {
                    overrideSteering.set(m, Optional.of(prevSetpoint.moduleStates()[m].angle));
                    continue;
                }

                var necessaryRotation = prevSetpoint
                        .moduleStates()[m]
                        .angle
                        .unaryMinus()
                        .rotateBy(desiredModuleStates[m].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.kPi);
                }
                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;

                if (numStepsNeeded <= 1.0) {
                    overrideSteering.set(m, Optional.of(desiredModuleStates[m].angle));
                } else {
                    overrideSteering.set(
                            m,
                            Optional.of(
                                    prevSetpoint.moduleStates()[m].angle.rotateBy(
                                            Rotation2d.fromRadians(
                                                    Math.signum(necessaryRotation.getRadians()) * max_theta_step
                                            )
                                    )
                            )
                    );
                    min_s = 0.0;
                }
                continue;
            }
            if (min_s == 0.0) {
                continue;
            }

            double maxHeadingChange =
                    (dt * wheelFrictionNewtons)
                            / ((massKG / moduleLocations.length) * Math.abs(prevSetpoint.moduleStates()[m].speedMetersPerSecond));
            max_theta_step = Math.min(max_theta_step, maxHeadingChange);

            double s = findSteeringMaxS(
                    prev_vx[m],
                    prev_vy[m],
                    prev_heading[m].getRadians(),
                    desired_vx[m],
                    desired_vy[m],
                    desired_heading[m].getRadians(),
                    max_theta_step
            );
            min_s = Math.min(min_s, s);
        }

        Translation2d chassisForceVec = new Translation2d();
        double chassisTorque = 0.0;
        for (int m = 0; m < moduleLocations.length; m++) {
            double lastVelRadPerSec = prevSetpoint.moduleStates()[m].speedMetersPerSecond / wheelRadiusMeters;
            double currentDraw = driveMotor.getCurrent(Math.abs(lastVelRadPerSec), inputVoltage);
            double reverseCurrentDraw = Math.abs(driveMotor.getCurrent(Math.abs(lastVelRadPerSec), -inputVoltage));
            currentDraw = Math.min(currentDraw, driveCurrentLimit);
            currentDraw = Math.max(currentDraw, 0);
            reverseCurrentDraw = Math.min(reverseCurrentDraw, driveCurrentLimit);
            reverseCurrentDraw = Math.max(reverseCurrentDraw, 0);
            double forwardModuleTorque = driveMotor.getTorque(currentDraw);
            double reverseModuleTorque = driveMotor.getTorque(reverseCurrentDraw);

            double prevSpeed = prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            desiredModuleStates[m].optimize(prevSetpoint.moduleStates()[m].angle);
            double desiredSpeed = desiredModuleStates[m].speedMetersPerSecond;

            int forceSign;
            Rotation2d forceAngle = prevSetpoint.moduleStates()[m].angle;
            double moduleTorque;
            if (epsilonEquals(prevSpeed, 0.0)
                    || (prevSpeed > 0 && desiredSpeed >= prevSpeed)
                    || (prevSpeed < 0 && desiredSpeed <= prevSpeed)) {
                moduleTorque = forwardModuleTorque;
                moduleTorque -= driveTorqueLoss;
                forceSign = 1;
                if (prevSpeed < 0) {
                    forceAngle = forceAngle.plus(Rotation2d.k180deg);
                }
            } else {
                moduleTorque = reverseModuleTorque;
                moduleTorque += driveTorqueLoss;
                forceSign = -1;
                if (prevSpeed > 0) {
                    forceAngle = forceAngle.plus(Rotation2d.k180deg);
                }
            }

            moduleTorque = Math.min(moduleTorque, driveMaxTorqueWithoutSlip);

            double forceAtCarpet = moduleTorque / wheelRadiusMeters;
            Translation2d moduleForceVec = new Translation2d(forceAtCarpet * forceSign, forceAngle);

            chassisForceVec = chassisForceVec.plus(moduleForceVec);

            if (!epsilonEquals(0, moduleForceVec.getNorm())) {
                Rotation2d angleToModule = moduleLocations[m].getAngle();
                Rotation2d theta = moduleForceVec.getAngle().minus(angleToModule);
                chassisTorque += forceAtCarpet * moduleDistances[m] * theta.getSin();
            }
        }

        Translation2d chassisAccelVec = chassisForceVec.div(massKG);
        double chassisAngularAccel = chassisTorque / MOIKGMetersSquared;

        if (constraints != null) {
            double linearAccel = chassisAccelVec.getNorm();
            if (linearAccel > constraints.maxAccelerationMetersPerSecSquared()) {
                chassisAccelVec = chassisAccelVec.times(constraints.maxAccelerationMetersPerSecSquared() / linearAccel);
            }
            chassisAngularAccel =
                    MathUtil.clamp(
                            chassisAngularAccel,
                            -constraints.maxAngularAccelerationRadPerSecSquared(),
                            constraints.maxAngularAccelerationRadPerSecSquared());
        }

        ChassisSpeeds chassisAccel = new ChassisSpeeds(chassisAccelVec.getX(), chassisAccelVec.getY(), chassisAngularAccel);
        var accelStates = kinematics.toSwerveModuleStates(chassisAccel);

        for (int m = 0; m < moduleLocations.length; m++) {
            if (min_s == 0.0) {
                break;
            }

            double maxVelStep = Math.abs(accelStates[m].speedMetersPerSecond * dt);

            double vx_min_s = min_s == 1.0
                    ? desired_vx[m]
                    : (desired_vx[m] - prev_vx[m]) * min_s + prev_vx[m];
            double vy_min_s = min_s == 1.0
                    ? desired_vy[m]
                    : (desired_vy[m] - prev_vy[m]) * min_s + prev_vy[m];
            double s = findDriveMaxS(prev_vx[m], prev_vy[m], vx_min_s, vy_min_s, maxVelStep);
            min_s = Math.min(min_s, s);
        }

        ChassisSpeeds retSpeeds = new ChassisSpeeds(
                prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond + min_s * dx,
                prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond + min_s * dy,
                prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond + min_s * dtheta
        );
        retSpeeds = ChassisSpeeds.discretize(retSpeeds, dt);

        double prevVelX = prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond;
        double prevVelY = prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond;
        double chassisAccelX = (retSpeeds.vxMetersPerSecond - prevVelX) / dt;
        double chassisAccelY = (retSpeeds.vyMetersPerSecond - prevVelY) / dt;
        double chassisForceX = chassisAccelX * massKG;
        double chassisForceY = chassisAccelY * massKG;

        double angularAccel = (retSpeeds.omegaRadiansPerSecond - prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond) / dt;
        double angTorque = angularAccel * MOIKGMetersSquared;
        ChassisSpeeds chassisForces = new ChassisSpeeds(chassisForceX, chassisForceY, angTorque);

        Translation2d[] wheelForces = chassisForcesToWheelForceVectors(chassisForces);

        var retStates = kinematics.toSwerveModuleStates(retSpeeds);
        double[] accelFF = new double[moduleLocations.length];
        double[] linearForceFF = new double[moduleLocations.length];
        double[] torqueCurrentFF = new double[moduleLocations.length];
        double[] forceXFF = new double[moduleLocations.length];
        double[] forceYFF = new double[moduleLocations.length];
        for (int m = 0; m < moduleLocations.length; m++) {
            double wheelForceDist = wheelForces[m].getNorm();
            double appliedForce =
                    greaterThanEpsilon(wheelForceDist)
                            ? wheelForceDist * wheelForces[m].getAngle().minus(retStates[m].angle).getCos()
                            : 0.0;
            double wheelTorque = appliedForce * wheelRadiusMeters;
            double torqueCurrent = driveMotor.getCurrent(wheelTorque);

            final var maybeOverride = overrideSteering.get(m);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(retStates[m].angle.unaryMinus().rotateBy(override))) {
                    retStates[m].speedMetersPerSecond *= -1.0;
                    appliedForce *= -1.0;
                    torqueCurrent *= -1.0;
                }
                retStates[m].angle = override;
            }
            final var deltaRotation =
                    prevSetpoint.moduleStates()[m].angle.unaryMinus().rotateBy(retStates[m].angle);
            if (flipHeading(deltaRotation)) {
                retStates[m].angle = retStates[m].angle.rotateBy(Rotation2d.k180deg);
                retStates[m].speedMetersPerSecond *= -1.0;
                appliedForce *= -1.0;
                torqueCurrent *= -1.0;
            }

            accelFF[m] = (retStates[m].speedMetersPerSecond - prevSetpoint.moduleStates()[m].speedMetersPerSecond) / dt;
            linearForceFF[m] = appliedForce;
            torqueCurrentFF[m] = torqueCurrent;
            forceXFF[m] = wheelForces[m].getX();
            forceYFF[m] = wheelForces[m].getY();
        }

        return new SwerveSetpoint(
                retSpeeds,
                retStates,
                new DriveFeedforwards(accelFF, linearForceFF, torqueCurrentFF, forceXFF, forceYFF)
        );
    }

    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredStateRobotRelative,
            final SwerveConstraints constraints,
            double dt
    ) {
        return generateSetpoint(
                prevSetpoint,
                desiredStateRobotRelative,
                constraints,
                dt,
                RobotController.getInputVoltage()
        );
    }

    public SwerveSetpoint generateSetpoint(final SwerveSetpoint prevSetpoint, ChassisSpeeds desiredStateRobotRelative, double dt) {
        return generateSetpoint(
                prevSetpoint,
                desiredStateRobotRelative,
                null,
                dt,
                RobotController.getInputVoltage()
        );
    }

    public SwerveSetpoint generateSetpoint(
            final SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredStateRobotRelative,
            double dt,
            double inputVoltage
    ) {
        return generateSetpoint(
                prevSetpoint,
                desiredStateRobotRelative,
                null,
                dt,
                inputVoltage
        );
    }

    private static boolean flipHeading(Rotation2d prevToGoal) {
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

    private static double findSteeringMaxS(
            double x_0,
            double y_0,
            double theta_0,
            double x_1,
            double y_1,
            double theta_1,
            double max_deviation) {
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
        return Double.isFinite(s) && s >= 0 && s <= 1;
    }

    private static double findDriveMaxS(
            double x_0, double y_0, double x_1, double y_1, double max_vel_step) {

        double l_0 = x_0 * x_0 + y_0 * y_0;
        double l_1 = x_1 * x_1 + y_1 * y_1;
        double sqrt_l_0 = Math.sqrt(l_0);
        double diff = Math.sqrt(l_1) - sqrt_l_0;
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

    public Translation2d[] chassisForcesToWheelForceVectors(ChassisSpeeds chassisForces) {
        var chassisForceVector = new SimpleMatrix(3, 1);
        chassisForceVector.setColumn(
                0,
                0,
                chassisForces.vxMetersPerSecond,
                chassisForces.vyMetersPerSecond,
                chassisForces.omegaRadiansPerSecond);

        var moduleForceMatrix = forceKinematics.mult(chassisForceVector.divide(moduleLocations.length));

        Translation2d[] forceVectors = new Translation2d[moduleLocations.length];
        for (int m = 0; m < moduleLocations.length; m++) {
            double x = moduleForceMatrix.get(m * 2, 0);
            double y = moduleForceMatrix.get(m * 2 + 1, 0);

            forceVectors[m] = new Translation2d(x, y);
        }

        return forceVectors;
    }
}
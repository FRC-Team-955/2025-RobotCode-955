package frc.robot.subsystems.drive;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.util.subsystem.SubsystemBaseExt;
import frc.robot.util.swerve.ModuleLimits;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.PhoenixOdometryThread.phoenixLock;
import static frc.robot.subsystems.drive.SparkOdometryThread.sparkLock;

public class Drive extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(ControlMode.OPEN_LOOP),
        WHEEL_RADIUS_CHARACTERIZATION(ControlMode.CLOSED_LOOP_DIRECT),
        IDLE(ControlMode.STOP),
        DRIVE_JOYSTICK(ControlMode.CLOSED_LOOP_OPTIMIZED),
        DRIVE_JOYSTICK_ASSISTED(ControlMode.CLOSED_LOOP_OPTIMIZED),
        MOVE_TO(ControlMode.CLOSED_LOOP_OPTIMIZED),
        FOLLOW_TRAJECTORY(ControlMode.CLOSED_LOOP_DIRECT);

        public final ControlMode controlMode;
    }

    public enum ControlMode {
        /** Open loop; no closed loop control will happen */
        OPEN_LOOP,
        /** ChassisSpeeds will be optimized with the setpoint generator (unless disabled) before being fed to modules */
        CLOSED_LOOP_OPTIMIZED,
        /** ChassisSpeeds will be directly fed to modules */
        CLOSED_LOOP_DIRECT,
        /** All modules will stop */
        STOP
    }

    @Getter
    private Goal goal = Goal.IDLE;
    private ChassisSpeeds closedLoopSetpoint;

    private Command withGoal(Goal goal, Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                Drive.this.goal = goal;
                super.initialize();
            }
        };
    }

    private final GyroIO gyroIO = DriveConstants.gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    /**
     * FL, FR, BL, BR
     */
    private final Module[] modules = new Module[4];
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    @Getter
    private Rotation2d rawGyroRotation = new Rotation2d();

    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(robotState.getKinematics());
    /** If null, it will be set to the measured ChassisSpeeds and module states when the setpoint generator starts to be used */
    private SwerveSetpoint prevSetpoint = null;

    public final SysIdRoutine sysId;

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);

    private final PIDController choreoFeedbackX = driveConfig.choreoFeedbackXY().toPID();
    private final PIDController choreoFeedbackY = driveConfig.choreoFeedbackXY().toPID();
    private final PIDController choreoFeedbackOmega = driveConfig.choreoFeedbackOmega().toPIDWrapRadians();

    private final PIDController moveToX = moveToXY.toPID();
    private final PIDController moveToY = moveToXY.toPID();
    private final PIDController moveToOmega = DriveConstants.moveToOmega.toPIDWrapRadians();

    private static Drive instance;

    public static Drive get() {
        if (instance == null)
            synchronized (Drive.class) {
                instance = new Drive();
            }

        return instance;
    }

    private Drive() {
        // Array is currently four nulls, so length works just fine
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(moduleIO[i], i);
        }

        // Usage reporting for swerve template
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();
        SparkOdometryThread.getInstance().start();

        sysId = Util.sysIdRoutine(
                "Drive",
                (voltage) -> {
                    for (var module : modules) {
                        module.runCharacterization(voltage.in(Volts));
                    }
                },
                () -> goal = Goal.CHARACTERIZATION,
                this
        );
    }

    @Override
    public void periodicBeforeCommands() {
        phoenixLock.lock();
        sparkLock.lock();

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);

        for (var module : modules) {
            module.periodicBeforeCommands();
        }

        phoenixLock.unlock();
        sparkLock.unlock();

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected);

        // Odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle
                );
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            // Sanity check that gyroInputs.odometryYawPositionsRad has an element at i
            // This might not always happen due to timing stuff
            // The modules themselves should be synced up though?
            if (gyroInputs.connected && gyroInputs.odometryYawPositionsRad.length > i) {
                // Use the real gyro angle
                rawGyroRotation = new Rotation2d(gyroInputs.odometryYawPositionsRad[i]);
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = robotState.getKinematics().toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            robotState.applyOdometryUpdate(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Drive/Goal", goal);

        // Stop moving when idle or disabled
        if (goal.controlMode == ControlMode.STOP || DriverStation.isDisabled()) {
            Logger.recordOutput("Drive/ClosedLoop", false);
            prevSetpoint = null;

            for (var module : modules) {
                module.stop();
            }
        }
        // Closed loop control
        else if ((goal.controlMode == ControlMode.CLOSED_LOOP_DIRECT
                || goal.controlMode == ControlMode.CLOSED_LOOP_OPTIMIZED
        )
                && closedLoopSetpoint != null
        ) {
            Logger.recordOutput("Drive/ClosedLoop", true);
            Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", closedLoopSetpoint);

            if (useSetpointGenerator && !disableDriving && goal.controlMode == ControlMode.CLOSED_LOOP_OPTIMIZED) {
                Logger.recordOutput("Drive/SetpointGenerator", true);

                Logger.recordOutput(
                        "Drive/ModuleStates/Setpoints",
                        // DON'T DO ANYTHING WITH THIS. SETPOINT GENERATOR SHOULD NOT GET A DISCRETIZED SETPOINT
                        // Only for logging
                        robotState.getKinematics().toSwerveModuleStates(
                                ChassisSpeeds.discretize(closedLoopSetpoint, 0.02)
                        )
                );

                if (prevSetpoint == null) {
                    // Reset to current chassis speeds and module states
                    prevSetpoint = new SwerveSetpoint(
                            getMeasuredChassisSpeeds(),
                            getMeasuredModuleStates()
                    );
                }

                prevSetpoint = setpointGenerator.generateSetpoint(
                        new ModuleLimits(
                                getSpeedScalar() * driveConfig.maxLinearSpeedMetersPerSec(),
                                getSpeedScalar() * driveConfig.maxLinearAccelMetersPerSecSquared(),
                                getSpeedScalar() * driveConfig.maxTurnVelocityRadPerSec()
                        ),
                        prevSetpoint,
                        closedLoopSetpoint, // THIS SHOULD NOT BE DISCRETIZED
                        0.02
                );
                var setpointStates = prevSetpoint.moduleStates();

                // Send setpoints to modules
                for (int i = 0; i < modules.length; i++) {
                    modules[i].runSetpointUnoptimized(setpointStates[i]);
                }

                // Log setpoint states
                Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
                Logger.recordOutput("Drive/ChassisSpeeds/SetpointOptimized", prevSetpoint.chassisSpeeds());
            } else {
                Logger.recordOutput("Drive/SetpointGenerator", false);

                // Calculate module setpoints
                ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(closedLoopSetpoint, 0.02);
                SwerveModuleState[] setpointStates = robotState.getKinematics().toSwerveModuleStates(discreteSpeeds);
                if (disableDriving) {
                    for (int i = 0; i < modules.length; i++) {
                        setpointStates[i].speedMetersPerSecond = 0.0;
                    }
                } else {
                    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, driveConfig.maxLinearSpeedMetersPerSec());
                }

                Logger.recordOutput("Drive/ModuleStates/Setpoints", setpointStates);

                // Send setpoints to modules
                for (int i = 0; i < modules.length; i++) {
                    // The module sets setpointStates[i] to the optimized setpoint, useful for logging
                    modules[i].runSetpointOptimized(setpointStates[i]);
                }

                // Log setpoint states
                Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
                Logger.recordOutput("Drive/ChassisSpeeds/SetpointOptimized", robotState.getKinematics().toChassisSpeeds(setpointStates));
            }
        } else {
            Logger.recordOutput("Drive/ClosedLoop", false);
            prevSetpoint = null;
        }

        // Run module closed loop control
        for (var module : modules) {
            module.periodicAfterCommands();
        }
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    private void stopWithX() {
        Rotation2d[] headings = new Rotation2d[modules.length];
        for (int i = 0; i < modules.length; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        // Why does this work? See SwerveDriveKinematics.toModuleStates
        robotState.getKinematics().resetHeadings(headings);
        closedLoopSetpoint = new ChassisSpeeds();
    }

    @AutoLogOutput(key = "Drive/ChassisSpeeds/Measured")
    private ChassisSpeeds getMeasuredChassisSpeeds() {
        return robotState.getKinematics().toChassisSpeeds(getMeasuredModuleStates());
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "Drive/ModuleStates/Measured")
    private SwerveModuleState[] getMeasuredModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    public SwerveModulePosition[] getMeasuredModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    public AutoFactory createAutoFactory() {
        return new AutoFactory(
                robotState::getPose,
                robotState::setPose,
                this::choreoController,
                true,
                this,
                this::choreoTrajectoryLogger
        );
    }

    private void choreoController(SwerveSample sample) {
        var currentPose = robotState.getPose();

        Logger.recordOutput("Drive/TrajectorySetpoint", sample.getPose());
        closedLoopSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                sample.vx + choreoFeedbackX.calculate(currentPose.getX(), sample.x),
                sample.vy + choreoFeedbackY.calculate(currentPose.getY(), sample.y),
                sample.omega + choreoFeedbackOmega.calculate(currentPose.getRotation().getRadians(), sample.heading),
                currentPose.getRotation() // Trajectories are absolute, don't flip
        );
    }

    private void choreoTrajectoryLogger(Trajectory<SwerveSample> trajectory, boolean running) {
        // This will run on initialize and end of the trajectory
        // follow command, so it's basically the same as wrapping
        // the trajectory command
        if (running) {
            goal = Goal.FOLLOW_TRAJECTORY;
            Logger.recordOutput("Drive/Trajectory", trajectory.getPoses());
        } else {
            goal = Goal.IDLE;
        }
    }

    private void runDrive(Translation2d linearVelocity, double omega) {
        // Convert to field relative speeds & send command
        closedLoopSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * driveConfig.maxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * driveConfig.maxLinearSpeedMetersPerSec(),
                omega * joystickMaxAngularSpeedRadPerSec,
                Util.flipIfNeeded(robotState.getRotation()) // Driver is alliance relative, flip
        );
    }

    private void runDriveAssisted(Pose2d assistPose, Translation2d linearVelocity, double linearMagnitude, double omegaMagnitude) {
        var currentPose = robotState.getPose();

        var driverX = linearVelocity.getX() * driveConfig.maxLinearSpeedMetersPerSec();
        var driverY = linearVelocity.getY() * driveConfig.maxLinearSpeedMetersPerSec();
        var driverOmega = omegaMagnitude * joystickMaxAngularSpeedRadPerSec;

        var assistX = moveToX.calculate(currentPose.getX(), assistPose.getX());
        assistX = MathUtil.clamp(assistX, -1, 1) // Limit to 100% max linear speed
                * driveConfig.maxLinearSpeedMetersPerSec()
                * linearMagnitude; // Limit to the driver's overall linear speed
        Logger.recordOutput("Drive/Assist/PID/X", assistX);

        var assistY = moveToY.calculate(currentPose.getY(), assistPose.getY());
        assistY = MathUtil.clamp(assistY, -1, 1) // Limit to 100% max linear speed
                * driveConfig.maxLinearSpeedMetersPerSec()
                * linearMagnitude; // Limit to the driver's overall linear speed
        Logger.recordOutput("Drive/Assist/PID/Y", assistY);

        var assistOmega = moveToOmega.calculate(currentPose.getRotation().getRadians(), assistPose.getRotation().getRadians());
        assistOmega = MathUtil.clamp(assistOmega, -1, 1) // Limit to 100% max angular speed
                * driveConfig.maxAngularSpeedRadPerSec()
                // If we are driving fast and not rotating, need fast rotation assist, so limit to driver's overall linear speed
                // Otherwise, limit to driver omega speed
                * Math.max(omegaMagnitude, linearMagnitude);
        Logger.recordOutput("Drive/Assist/PID/Omega", assistOmega);

        closedLoopSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                0.75 * assistX,
                0.75 * assistY,
                0.75 * assistOmega,
                currentPose.getRotation() // Move to is absolute, don't flip
        ).plus(ChassisSpeeds.fromFieldRelativeSpeeds(
                0.25 * driverX,
                0.25 * driverY,
                0.25 * driverOmega,
                Util.flipIfNeeded(currentPose.getRotation()) // Driver is alliance relative, flip
        ));
    }

    private void runMoveTo(Pose2d pose) {
        var currentPose = robotState.getPose();

        closedLoopSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                // Limit to 100% max linear speed
                MathUtil.clamp(moveToX.calculate(currentPose.getX(), pose.getX()), -1, 1)
                        * driveConfig.maxLinearSpeedMetersPerSec(),
                // Limit to 100% max linear speed
                MathUtil.clamp(moveToY.calculate(currentPose.getY(), pose.getY()), -1, 1)
                        * driveConfig.maxLinearSpeedMetersPerSec(),
                // Limit to 100% max angular speed
                MathUtil.clamp(moveToOmega.calculate(currentPose.getRotation().getRadians(), pose.getRotation().getRadians()), -1, 1)
                        * driveConfig.maxAngularSpeedRadPerSec(),
                currentPose.getRotation() // Move to is absolute, don't flip
        );
    }

    public Command moveTo(Supplier<Pose2d> poseSupplier) {
        return withGoal(
                Goal.MOVE_TO,
                run(() -> runMoveTo(poseSupplier.get()))
        ).withName("Drive Move To");
    }

    public Command driveJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, Supplier<Optional<Pose2d>> assistPoseSupplier) {
        return run(() -> {
            // Reset goal from assisted every loop
            goal = Goal.DRIVE_JOYSTICK;

            // Joystick inputs
            var x = xSupplier.getAsDouble();
            var y = ySupplier.getAsDouble();
            var omega = omegaSupplier.getAsDouble();

            Logger.recordOutput("Drive/JoystickDrive/Suppliers/X", x);
            Logger.recordOutput("Drive/JoystickDrive/Suppliers/Y", y);
            Logger.recordOutput("Drive/JoystickDrive/Suppliers/Omega", omega);

            // Calculate linear velocity and omega from joystick inputs
            var linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), joystickDriveDeadband);
            linearMagnitude = linearMagnitude * linearMagnitude;

            var joystickLinearDirection = new Rotation2d(x, y);
            var linearVelocity = new Pose2d(new Translation2d(), joystickLinearDirection)
                    .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                    .getTranslation();

            var omegaMagnitude = MathUtil.applyDeadband(omega, joystickDriveDeadband);
            omegaMagnitude = Math.copySign(omegaMagnitude * omegaMagnitude, omegaMagnitude);

            Logger.recordOutput("Drive/JoystickDrive/LinearMagnitude", linearMagnitude);
            Logger.recordOutput("Drive/JoystickDrive/LinearDirection", joystickLinearDirection);
            Logger.recordOutput("Drive/JoystickDrive/LinearVelocity", linearVelocity);
            Logger.recordOutput("Drive/JoystickDrive/OmegaMagnitude", omegaMagnitude);

            var optionalAssistPose = assistPoseSupplier.get();
            if (optionalAssistPose.isPresent()) {
                // Mark assist pose as present
                Logger.recordOutput("Drive/Assist/Present", true);

                var currentPose = robotState.getPose();
                var assistPose = optionalAssistPose.get();
                Logger.recordOutput("Drive/Assist/Pose", assistPose);

                // Get the translation between robot and assist
                var robotToAssist = assistPose.getTranslation().minus(currentPose.getTranslation());
                // Calculate direction from robot to assist
                var robotToAssistDirection = new Rotation2d(robotToAssist.getX(), robotToAssist.getY());
                Logger.recordOutput("Drive/Assist/RobotToAssistDirection", robotToAssistDirection);

                // Flip joystick direction to match robot to assist direction
                // Joystick direction is relative to alliance wall and needs to be flipped on red alliance to match origin
                var joystickLinearDirectionFlipped = Util.flipIfNeeded(joystickLinearDirection);
                Logger.recordOutput("Drive/Assist/FlippedJoystickLinearDirection", joystickLinearDirectionFlipped);

                // Get difference between joystick direction and assist direction
                var directionDiff = robotToAssistDirection.minus(joystickLinearDirectionFlipped);
                Logger.recordOutput("Drive/Assist/DirectionDifference", directionDiff);

                // Get distance to assist pose
                var distanceToAssist = currentPose.getTranslation().getDistance(assistPose.getTranslation());
                Logger.recordOutput("Drive/Assist/DistanceToAssist", distanceToAssist);

                // If we are:
                if (
                    // - above linear joystick deadband (so we are moving linearly in some way - if we are only rotating, don't assist)
                        (Math.abs(x) > joystickDriveDeadband || Math.abs(y) > joystickDriveDeadband) &&
                                // - going towards the assist pose based on threshold
                                Math.abs(directionDiff.getRadians()) < assistDirectionToleranceRad &&
                                // - close enough to assist pose
                                distanceToAssist < assistMaximumDistanceMeters
                ) {
                    // then use automatic control.
                    Logger.recordOutput("Drive/Assist/Running", true);
                    goal = Goal.DRIVE_JOYSTICK_ASSISTED;
                    runDriveAssisted(assistPose, linearVelocity, linearMagnitude, omegaMagnitude);
                } else {
                    Logger.recordOutput("Drive/Assist/Running", false);
                    runDrive(linearVelocity, omegaMagnitude);
                }
            } else {
                Logger.recordOutput("Drive/Assist/Present", false);
                Logger.recordOutput("Drive/Assist/Running", false);
                runDrive(linearVelocity, omegaMagnitude);
            }
        }).withName("Drive Joystick");
    }

    public Command wheelRadiusCharacterization(WheelRadiusCharacterization.Direction direction) {
        return withGoal(Goal.WHEEL_RADIUS_CHARACTERIZATION, new WheelRadiusCharacterization(direction)).withName("Drive Wheel Radius Characterization");
    }

    public class WheelRadiusCharacterization extends Command {
        private static final LoggedNetworkNumber characterizationSpeedRadPerSec = new LoggedNetworkNumber("Drive/Wheel Radius Characterization Rotation Speed (rad per sec)", 1.0);

        @RequiredArgsConstructor
        public enum Direction {
            CLOCKWISE(-1),
            COUNTER_CLOCKWISE(1);

            private final int value;
        }

        private final Direction omegaDirection;
        private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

        private double lastGyroYawRads = 0.0;
        private double accumGyroYawRads = 0.0;

        private double[] startWheelPositions;

        private double currentEffectiveWheelRadius = 0.0;

        private WheelRadiusCharacterization(Direction omegaDirection) {
            this.omegaDirection = omegaDirection;
            addRequirements(Drive.this);
        }

        private double[] getWheelRadiusCharacterizationPositions() {
            return Arrays.stream(modules).mapToDouble(Module::getPositionRad).toArray();
        }

        @Override
        public void initialize() {
            // Reset
            lastGyroYawRads = rawGyroRotation.getRadians();
            accumGyroYawRads = 0.0;

            startWheelPositions = getWheelRadiusCharacterizationPositions();

            omegaLimiter.reset(0);
        }

        @Override
        public void execute() {
            // Run drive at velocity
            var omega = omegaLimiter.calculate(omegaDirection.value * characterizationSpeedRadPerSec.get());
            closedLoopSetpoint = new ChassisSpeeds(0, 0, omega);

            // Get yaw and wheel positions
            accumGyroYawRads += MathUtil.angleModulus(rawGyroRotation.getRadians() - lastGyroYawRads);
            lastGyroYawRads = rawGyroRotation.getRadians();
            double averageWheelPosition = 0.0;
            double[] wheelPositions = getWheelRadiusCharacterizationPositions();
            for (int i = 0; i < modules.length; i++) {
                averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
            }
            averageWheelPosition /= modules.length;

            currentEffectiveWheelRadius = (accumGyroYawRads * drivebaseRadius) / averageWheelPosition;
            Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
            Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
            Logger.recordOutput(
                    "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
                    Units.metersToInches(currentEffectiveWheelRadius)
            );
        }

        @Override
        public void end(boolean interrupted) {
            if (accumGyroYawRads <= Math.PI * 2.0) {
                System.out.println("Not enough data for characterization");
            } else {
                System.out.println("Effective Wheel Radius: " + Units.metersToInches(currentEffectiveWheelRadius) + " inches");
            }
        }
    }

}

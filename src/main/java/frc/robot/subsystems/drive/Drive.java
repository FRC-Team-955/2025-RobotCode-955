package frc.robot.subsystems.drive;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.characterization.FeedforwardCharacterization;
import frc.lib.commands.CommandsExt;
import frc.lib.subsystem.Periodic;
import frc.lib.swerve.ModuleLimits;
import frc.lib.swerve.SwerveSetpoint;
import frc.lib.swerve.SwerveSetpointGenerator;
import frc.robot.OperatorDashboard;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.subsystems.drive.goals.DriveJoystickGoal;
import frc.robot.subsystems.drive.goals.MoveToGoal;
import frc.robot.subsystems.drive.goals.VelocityRobotRelativeGoal;
import frc.robot.subsystems.elevator.Elevator;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.lib.HighFrequencySamplingThread.highFrequencyLock;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.DriveTuning.*;

public class Drive implements Periodic {
    private final RobotState robotState = RobotState.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Elevator elevator = Elevator.get();

    private final GyroIO gyroIO = createGyroIO();
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(ControlMode.OPEN_LOOP),
        WHEEL_RADIUS_CHARACTERIZATION(ControlMode.CLOSED_LOOP_DIRECT),
        IDLE(ControlMode.STOP),
        DRIVE_JOYSTICK(ControlMode.CLOSED_LOOP_OPTIMIZED),
        DRIVE_JOYSTICK_ASSISTED(ControlMode.CLOSED_LOOP_OPTIMIZED),
        MOVE_TO(ControlMode.CLOSED_LOOP_OPTIMIZED),
        MOVE_TO_DRIVE_JOYSTICK_MERGED(ControlMode.CLOSED_LOOP_OPTIMIZED),
        FOLLOW_TRAJECTORY(ControlMode.CLOSED_LOOP_DIRECT),
        VELOCITY_ROBOT_RELATIVE(ControlMode.CLOSED_LOOP_OPTIMIZED);

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

    private static Drive instance;

    public static Drive get() {
        if (instance == null)
            synchronized (Drive.class) {
                instance = new Drive();
            }

        return instance;
    }

    private Drive() {
        var moduleIO = createModuleIO();
        // Array is currently four nulls, so length works just fine
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new Module(moduleIO[i], i);
        }

        // Usage reporting for swerve template
        HAL.report(FRCNetComm.tResourceType.kResourceType_RobotDrive, FRCNetComm.tInstances.kRobotDriveSwerve_AdvantageKit);

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
        highFrequencyLock.lock();

        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Inputs/Drive/Gyro", gyroInputs);

        for (var module : modules) {
            module.updateAndProcessInputs();
        }

        highFrequencyLock.unlock();

        for (var module : modules) {
            module.periodicBeforeCommands();
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected);

        // Odometry
        if (useHighFrequencyOdometry) {
            // All timestamps will be synced by HighFrequencySamplingThread
            double[] sampleTimestamps = modules[0].getOdometryTimestamps();
            boolean anySampleDiscarded = false;
            for (int sample = 0; sample < sampleTimestamps.length; sample++) {
                double sampleTimestamp = sampleTimestamps[sample];
                boolean discardSample = false;

                // Read wheel positions and deltas from each module
                SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
                SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
                for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                    double positionMeters = modules[moduleIndex].getOdometryDrivePositionsRad()[sample] * driveConfig.wheelRadiusMeters();
                    Rotation2d angle = new Rotation2d(modules[moduleIndex].getOdometryTurnPositionsRad()[sample]);
                    var modulePosition = new SwerveModulePosition(positionMeters, angle);

                    modulePositions[moduleIndex] = modulePosition;
                    moduleDeltas[moduleIndex] = new SwerveModulePosition(
                            modulePosition.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                            modulePosition.angle
                    );
                    lastModulePositions[moduleIndex] = modulePosition;

                    // We actually don't really care if one of the motors is disconnected, because odometry
                    // can handle one wheel position isn't changing. The issue is when one wheel changes drastically
                    if (Math.abs(moduleDeltas[moduleIndex].distanceMeters) > odometryPositionDeltaDiscardMeters) {
                        discardSample = true;
                    }
                }

                // Update gyro angle
                // Sanity check in case gyro is connected but not giving timestamps
                if (gyroInputs.connected && !disableGyro && gyroInputs.odometryYawTimestamps.length > sample) {
                    // Use the real gyro angle
                    rawGyroRotation = new Rotation2d(gyroInputs.odometryYawPositionsRad[sample]);
                } else {
                    // Use the angle delta from the kinematics and module deltas
                    Twist2d twist = robotState.getKinematics().toTwist2d(moduleDeltas);
                    rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
                }

                // Apply update
                if (discardSample) {
                    anySampleDiscarded = true;
                    // If we need to discard it, apply the update and then revert the pose back to the pose before applying the update
                    // This means that the previous wheel positions stored by odometry will be updated to the new wheel positions,
                    // but the pose won't change
                    Pose2d prevPose = robotState.getPose();
                    robotState.applyOdometryUpdate(sampleTimestamp, rawGyroRotation, modulePositions);
                    robotState.setPose(prevPose);
                } else {
                    robotState.applyOdometryUpdate(sampleTimestamp, rawGyroRotation, modulePositions);
                }
            }
            Logger.recordOutput("Drive/SampleDiscarded", anySampleDiscarded);
        } else {
            boolean discardSample = false;

            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[modules.length];
            for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                double positionMeters = modules[moduleIndex].getDrivePositionRad() * driveConfig.wheelRadiusMeters();
                Rotation2d angle = modules[moduleIndex].getTurnAngle();
                var modulePosition = new SwerveModulePosition(positionMeters, angle);

                modulePositions[moduleIndex] = modulePosition;
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePosition.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePosition.angle
                );
                lastModulePositions[moduleIndex] = modulePosition;

                // We actually don't really care if one of the motors is disconnected, because odometry
                // can handle one wheel position isn't changing. The issue is when one wheel changes drastically
                if (Math.abs(moduleDeltas[moduleIndex].distanceMeters) > odometryPositionDeltaDiscardMeters) {
                    discardSample = true;
                }
            }

            // Update gyro angle
            if (gyroInputs.connected && !disableGyro) {
                // Use the real gyro angle
                rawGyroRotation = new Rotation2d(gyroInputs.yawPositionRad);
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = robotState.getKinematics().toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            if (discardSample) {
                // If we need to discard it, apply the update and then revert the pose back to the pose before applying the update
                // This means that the previous wheel positions stored by odometry will be updated to the new wheel positions,
                // but the pose won't change
                Pose2d prevPose = robotState.getPose();
                robotState.applyOdometryUpdate(Timer.getTimestamp(), rawGyroRotation, modulePositions);
                robotState.setPose(prevPose);
            } else {
                robotState.applyOdometryUpdate(Timer.getTimestamp(), rawGyroRotation, modulePositions);
            }
            Logger.recordOutput("Drive/SampleDiscarded", discardSample);
        }

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            for (var module : modules) {
                module.setBrakeMode(!operatorDashboard.coastOverride.get());
            }
        }

        moduleDriveGainsTunable.ifChanged(gains -> {
            for (var module : modules) {
                module.setDrivePIDF(gains);
            }
        });
        moduleTurnGainsTunable.ifChanged(gains -> {
            for (var module : modules) {
                module.setTurnPIDF(gains);
            }
        });
    }

    @Override
    public void periodicAfterCommands() {
        ChassisSpeeds closedLoopSetpoint = null;
        switch (goal) {
            case CHARACTERIZATION -> {
                closedLoopSetpoint = null;
            }
            case WHEEL_RADIUS_CHARACTERIZATION -> {
                closedLoopSetpoint = null;
            }
            case IDLE -> closedLoopSetpoint = null;
            case DRIVE_JOYSTICK, DRIVE_JOYSTICK_ASSISTED -> {
                var output = DriveJoystickGoal.get();
                closedLoopSetpoint = output.getFirst();
                goal = output.getSecond();
            }
            case MOVE_TO, MOVE_TO_DRIVE_JOYSTICK_MERGED -> {
                var output = MoveToGoal.get();
                closedLoopSetpoint = output.getFirst();
                goal = output.getSecond();
            }
            case FOLLOW_TRAJECTORY -> {
                closedLoopSetpoint = null;
            }
            case VELOCITY_ROBOT_RELATIVE -> {
                closedLoopSetpoint = null;
            }
        }

        // Goal processing might change the goal
        Logger.recordOutput("Drive/Goal", goal);
        Logger.recordOutput("Drive/ControlMode", goal.controlMode);

        // Stop moving when idle or disabled
        if (goal.controlMode == ControlMode.STOP || DriverStation.isDisabled()) {
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
                        getModuleLimits(),
                        prevSetpoint,
                        closedLoopSetpoint, // THIS SHOULD NOT BE DISCRETIZED
                        0.02
                );
                var setpointStates = prevSetpoint.moduleStates();

                // Send setpoints to modules
                for (int i = 0; i < modules.length; i++) {
                    // The module sets setpointStates[i] to the cosine scaled setpoint, useful for logging
                    modules[i].runSetpoint(setpointStates[i], false);
                }

                // Log setpoint states
                Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
                Logger.recordOutput("Drive/ChassisSpeeds/SetpointOptimized", prevSetpoint.chassisSpeeds());
            } else {
                Logger.recordOutput("Drive/SetpointGenerator", false);
                prevSetpoint = null;

                // Calculate module setpoints
                ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(closedLoopSetpoint, 0.02);
                SwerveModuleState[] setpointStates = robotState.getKinematics().toSwerveModuleStates(discreteSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, driveConfig.moduleLimits().maxDriveVelocityMetersPerSec());

                Logger.recordOutput("Drive/ModuleStates/Setpoints", setpointStates);

                // Send setpoints to modules
                for (int i = 0; i < modules.length; i++) {
                    // The module sets setpointStates[i] to the optimized setpoint, useful for logging
                    modules[i].runSetpoint(setpointStates[i], true);
                }

                // Log setpoint states
                Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
                Logger.recordOutput("Drive/ChassisSpeeds/SetpointOptimized", robotState.getKinematics().toChassisSpeeds(setpointStates));
            }
        } else {
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
        // TODO goal or something
//        Rotation2d[] headings = new Rotation2d[modules.length];
//        for (int i = 0; i < modules.length; i++) {
//            headings[i] = moduleTranslations[i].getAngle();
//        }
//        // Why does this work? See SwerveDriveKinematics.toModuleStates
//        robotState.getKinematics().resetHeadings(headings);
//        closedLoopSetpoint = new ChassisSpeeds();
    }

    @AutoLogOutput(key = "Drive/ChassisSpeeds/Measured")
    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return robotState.getKinematics().toChassisSpeeds(getMeasuredModuleStates());
    }

    public ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                robotState.getKinematics().toChassisSpeeds(getMeasuredModuleStates()),
                robotState.getRotation() // Field is absolute, don't flip
        );
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

    public ModuleLimits getModuleLimits() {
        if (goal == Goal.MOVE_TO || goal == Goal.MOVE_TO_DRIVE_JOYSTICK_MERGED) {
            return moveToModuleLimits;
        }

        if (operatorDashboard.coralStuckInRobotMode.get()) {
            return driveConfig.moduleLimits();
        }

        return driveConfig.moduleLimits().times(elevator.getDriveConstraintScalar());
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

    public Command moveTo(Supplier<Pose2d> poseSupplier, BooleanSupplier mergeJoystickDrive) {
        return Commands.runOnce(() -> {
            MoveToGoal.setPoseSupplier(poseSupplier);
            MoveToGoal.setMergeJoystickDrive(mergeJoystickDrive);
            goal = Goal.MOVE_TO;
        });
    }

    public Command driveJoystick(Supplier<Optional<Pose2d>> assistPoseSupplier) {
        return Commands.runOnce(() -> {
            DriveJoystickGoal.setAssistPoseSupplier(assistPoseSupplier);
            goal = Goal.DRIVE_JOYSTICK;
        });
    }

    public Command runRobotRelative(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return Commands.runOnce(() -> {
            VelocityRobotRelativeGoal.setChassisSpeedsSupplier(chassisSpeedsSupplier);
            goal = Goal.VELOCITY_ROBOT_RELATIVE;
        });
    }

    public Command feedforwardCharacterization() {
        return withGoal(Goal.CHARACTERIZATION, new FeedforwardCharacterization(
                volts -> {
                    for (var module : modules) {
                        module.runCharacterization(volts);
                    }
                },
                () -> Arrays.stream(modules)
                        .mapToDouble(Module::getDriveVelocityRadPerSec)
                        .toArray(),
                modules.length,
                this
        ));
    }

    public Command fullSpeedCharacterization() {
        return withGoal(Goal.CHARACTERIZATION, CommandsExt.eagerSequence(
                startIdle(
                        () -> {
                            for (var module : modules) {
                                module.runCharacterization(2.0);
                            }
                        }
                ).withTimeout(2),
                startEnd(
                        () -> {
                            for (var module : modules) {
                                module.runCharacterization(12.0);
                            }
                        },
                        () -> {
                            for (var module : modules) {
                                module.runCharacterization(0.0);
                            }
                        }
                )
        ));
    }

    public Command wheelRadiusCharacterization(WheelRadiusCharacterization.Direction direction) {
        return withGoal(Goal.WHEEL_RADIUS_CHARACTERIZATION, new WheelRadiusCharacterization(direction));
    }

    public class WheelRadiusCharacterization extends Command {
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
            return Arrays.stream(modules).mapToDouble(Module::getDrivePositionRad).toArray();
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

            currentEffectiveWheelRadius = (accumGyroYawRads * drivebaseRadiusMeters) / averageWheelPosition;
            Logger.recordOutput("Drive/WheelRadiusCharacterization/DrivePosition", averageWheelPosition);
            Logger.recordOutput("Drive/WheelRadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
            Logger.recordOutput(
                    "Drive/WheelRadiusCharacterization/CurrentWheelRadiusInches",
                    Units.metersToInches(currentEffectiveWheelRadius)
            );
        }

        @Override
        public void end(boolean interrupted) {
            if (Math.abs(accumGyroYawRads) <= Math.PI * 2.0) {
                System.out.println("Not enough data for characterization");
            } else {
                System.out.println("Effective Wheel Radius: " + Units.metersToInches(currentEffectiveWheelRadius) + " inches");
            }
        }
    }

}

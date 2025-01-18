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
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.util.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.DriveDashboard.disableDriving;
import static frc.robot.subsystems.drive.PhoenixOdometryThread.phoenixLock;
import static frc.robot.subsystems.drive.SparkOdometryThread.sparkLock;

public class Drive extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();

    public enum Goal {
        CHARACTERIZATION,
        WHEEL_RADIUS_CHARACTERIZATION,
        IDLE,
        DRIVE_JOYSTICK,
        POINT_TOWARDS,
        FOLLOW_TRAJECTORY
    }

    private static final double JOYSTICK_DRIVE_DEADBAND = 0.1;

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

    public final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", Alert.AlertType.kError);

    private Goal goal = Goal.IDLE;
    private ChassisSpeeds closedLoopSetpoint;

    private final PIDController choreoFeedbackX = driveConfig.choreoFeedbackXY().toPID();
    private final PIDController choreoFeedbackY = driveConfig.choreoFeedbackXY().toPID();
    private final PIDController choreoFeedbackTheta = driveConfig.choreoFeedbackTheta().toPID();

    private Command withGoal(Goal newGoal, Command command) {
        return new WrapperCommand(command) {
            @Override
            public void initialize() {
                goal = newGoal;
                super.initialize();
            }
        };
    }

    private static Drive instance;

    public static Drive get() {
        if (instance == null)
            synchronized (Drive.class) {
                instance = new Drive();
            }

        return instance;
    }

    private Drive() {
        for (int i = 0; i < 4; i++) {
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
                    for (int i = 0; i < 4; i++) {
                        modules[i].runCharacterization(voltage.in(Volts));
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
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.mode != Constants.Mode.SIM);

        // Odometry
        double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle
                );
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
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
        if (goal == Goal.IDLE || DriverStation.isDisabled()) {
            Logger.recordOutput("Drive/ClosedLoop", false);
            for (var module : modules) {
                module.stop();
            }
            Logger.recordOutput("Drive/ModuleStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", new SwerveModuleState[]{});
        }
        // Closed loop control
        else if (goal != Goal.CHARACTERIZATION && closedLoopSetpoint != null) {
            Logger.recordOutput("Drive/ClosedLoop", true);
            Logger.recordOutput("Drive/ChassisSpeeds/Setpoint", closedLoopSetpoint);

            // Calculate module setpoints
            ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(closedLoopSetpoint, 0.02);
            SwerveModuleState[] setpointStates = robotState.getKinematics().toSwerveModuleStates(discreteSpeeds);
            if (disableDriving.get()) {
                for (int i = 0; i < 4; i++) {
                    setpointStates[i].speedMetersPerSecond = 0.0;
                }
            } else {
                SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, driveConfig.maxLinearSpeedMetersPerSec());
            }
            Logger.recordOutput("Drive/ModuleStates/Setpoints", setpointStates);

            // Send setpoints to modules
            for (int i = 0; i < 4; i++) {
                // The module set setpointStates[i] to the optimized setpoint, useful for logging
                modules[i].runSetpoint(setpointStates[i]);
            }

            // Log setpoint states
            Logger.recordOutput("Drive/ModuleStates/SetpointsOptimized", setpointStates);
        } else {
            Logger.recordOutput("Drive/ClosedLoop", false);
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
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = moduleTranslations[i].getAngle();
        }
        // Why does this work? See SwerveDriveKinematics.toModuleStates
        robotState.getKinematics().resetHeadings(headings);
        closedLoopSetpoint = new ChassisSpeeds();
    }

    @AutoLogOutput(key = "Drive/ChassisSpeeds/Measured")
    private ChassisSpeeds getMeasuredChassisSpeeds() {
        return robotState.getKinematics().toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "Drive/ModuleStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
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
                sample.omega + choreoFeedbackTheta.calculate(currentPose.getRotation().getRadians(), sample.heading),
                currentPose.getRotation()
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

    private void runDrive(double linearMagnitude, Rotation2d linearDirection, double omega) {
        // Calculate new linear velocity
        Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();

        // Convert to field relative speeds & send command
        closedLoopSetpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                linearVelocity.getX() * driveConfig.maxLinearSpeedMetersPerSec(),
                linearVelocity.getY() * driveConfig.maxLinearSpeedMetersPerSec(),
                omega * joystickMaxAngularSpeedRadPerSec,
                Util.shouldFlip()
                        ? robotState.getRotation()
                        : robotState.getRotation().plus(new Rotation2d(Math.PI))
        );
    }

    private double calculateLinearMagnitude(double x, double y) {
        var linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), JOYSTICK_DRIVE_DEADBAND);
        return linearMagnitude * linearMagnitude;
    }

    private Rotation2d calculateLinearDirection(double x, double y) {
        return new Rotation2d(x, y);
    }

    private double calculateOmega(double omega) {
        double omegaWithDeadband = MathUtil.applyDeadband(omega, JOYSTICK_DRIVE_DEADBAND);
        return Math.copySign(omegaWithDeadband * omegaWithDeadband, omegaWithDeadband);
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public Command driveJoystick(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        var cmd = run(() -> {
            var x = xSupplier.getAsDouble();
            var y = ySupplier.getAsDouble();
            var omega = omegaSupplier.getAsDouble();

            runDrive(
                    calculateLinearMagnitude(x, y),
                    calculateLinearDirection(x, y),
                    calculateOmega(omega)
            );
        });
        return withGoal(Goal.DRIVE_JOYSTICK, cmd).withName("Drive Joystick");
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
            for (int i = 0; i < 4; i++) {
                averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
            }
            averageWheelPosition /= 4.0;

            currentEffectiveWheelRadius = (accumGyroYawRads * DriveConstants.drivebaseRadius) / averageWheelPosition;
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

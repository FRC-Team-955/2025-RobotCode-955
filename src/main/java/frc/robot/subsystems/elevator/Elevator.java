package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.ReefAlign;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.RobotMechanism.middleOfRobot;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorTuning.*;

public class Elevator extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final ElevatorIO io = createIO();
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(null, false),
        STOW(stowGoalSetpoint::get, false), // Setpoint for when coral stuck in robot mode is activated is in periodicAfterCommands
        SCORE_L1(scoreL1GoalSetpoint::get, false),
        SCORE_L2(scoreL2GoalSetpoint::get, true),
        SCORE_L3(scoreL3GoalSetpoint::get, true),
        SCORE_L4(scoreL4GoalSetpoint::get, true),
        DESCORE_L2(descoreL2GoalSetpoint::get, false),
        DESCORE_L3(descoreL3GoalSetpoint::get, false),
        ZERO_CORAL(null, false),
        ZERO_ELEVATOR(null, false);

        /** Should be constant for every loop cycle */
        public final DoubleSupplier setpointMeters;
        private final boolean adjustForScoring;
    }

    @Getter
    private Goal goal = Goal.STOW;

    @AutoLogOutput(key = "Elevator/HasZeroed")
    private boolean hasZeroed = false;

    private boolean autoStop = false;
    private final Timer autoStopTimer = new Timer();
    private boolean prevEmergencyStopped = false;

    /** NOTE: UNITS IN METERS! */
    private TrapezoidProfile profileFullVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSquared
    ));
    private TrapezoidProfile profileGentleVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            gentleMaxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSquared
    ));
    private TrapezoidProfile.State previousStateMeters = null;

    @AutoLogOutput(key = "Elevator/DistanceFromScoringPositionMeters")
    private double distanceFromScoringPositionMeters = 0.0;

    private boolean manualCurrentLimitApplied = false;
    private double manualVoltage = gains.kG();

    private final Alert emergencyStoppedAlert = new Alert("Elevator is emergency stopped.", Alert.AlertType.kError);
    private final Alert notZeroedAlert = new Alert("Elevator is not zeroed! Please zero.", Alert.AlertType.kError);
    private final Alert leaderDisconnectedAlert = new Alert("Elevator leader motor is disconnected.", Alert.AlertType.kError);
    private final Alert followerDisconnectedAlert = new Alert("Elevator follower motor is disconnected.", Alert.AlertType.kError);
    private final Alert offsetSetAlert = new Alert("Elevator offset is not zero, bad things may happen.", Alert.AlertType.kWarning);
    private final Alert temperatureAlert = new Alert("Elevator motor temperature is high.", Alert.AlertType.kWarning);

    private static Elevator instance;

    public static Elevator get() {
        if (instance == null)
            synchronized (Elevator.class) {
                instance = new Elevator();
            }

        return instance;
    }

    private Elevator() {
        super(10);
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);

        leaderDisconnectedAlert.set(!inputs.leaderConnected);
        followerDisconnectedAlert.set(!inputs.followerConnected);

        temperatureAlert.set(Math.max(inputs.leaderTemperatureCelsius, inputs.followerTemperatureCelsius) > 60);

        // Check emergency stop and limits for auto stop
        var positionMeters = getPositionMeters();
        var velocityMetersPerSec = getVelocityMetersPerSec();
        if (!autoStop) {
            autoStop =
                    (positionMeters > upperLimit.positionMeters()
                            && velocityMetersPerSec > upperLimit.velocityMetersPerSec()
                    ) || (positionMeters < lowerLimit.positionMeters()
                            && velocityMetersPerSec < lowerLimit.velocityMetersPerSec());

            if (autoStop) {
                autoStopTimer.restart();
            }
        } else if (autoStopTimer.hasElapsed(0.75) && Math.abs(velocityMetersPerSec) < 0.5) {
            // Only disable auto stop if we have stopped for a bit (roughly - we don't want to get stuck in auto stop)
            autoStop = false;
        }
        Logger.recordOutput("Elevator/AutoStop", autoStop);

        boolean emergencyStopped = operatorDashboard.elevatorEStop.get() || autoStop;
        if (emergencyStopped != prevEmergencyStopped) {
            if (emergencyStopped) {
                System.out.println("Elevator is emergency stopping");
            } else {
                System.out.println("Elevator is no longer emergency stopped");
            }
            io.setEmergencyStopped(emergencyStopped);
            prevEmergencyStopped = emergencyStopped;
        }
        emergencyStoppedAlert.set(emergencyStopped);
        Logger.recordOutput("Elevator/EmergencyStop", emergencyStopped);

        // Update mechanisms
        robotMechanism.elevator.stage1Root.setPosition(middleOfRobot - Units.inchesToMeters(7) + 0.04, Units.inchesToMeters(2.85) + getPositionMeters() / 3);
        robotMechanism.elevator.stage2Root.setPosition(middleOfRobot - Units.inchesToMeters(7) + 0.02, Units.inchesToMeters(3.85) + getPositionMeters() / 3 * 2);
        robotMechanism.elevator.stage3Root.setPosition(middleOfRobot - Units.inchesToMeters(7), Units.inchesToMeters(4.85) + getPositionMeters());

        var endEffectorX = middleOfRobot - Units.inchesToMeters(11);
        var endEffectorY = Units.inchesToMeters(7) + getPositionMeters();
        robotMechanism.endEffector.root.setPosition(endEffectorX, endEffectorY);
        robotMechanism.endEffector.topRollersRoot.setPosition(endEffectorX - Units.inchesToMeters(3), endEffectorY + Units.inchesToMeters(10));

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        gainsTunable.ifChanged(io::setPIDF);

        if (maxVelocityMetersPerSecondTunable.hasChanged()
                || maxAccelerationMetersPerSecondSquaredTunable.hasChanged()
        ) {
            profileFullVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    maxVelocityMetersPerSecondTunable.get(),
                    maxAccelerationMetersPerSecondSquaredTunable.get()
            ));
            profileGentleVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    gentleMaxVelocityMetersPerSecond,
                    maxAccelerationMetersPerSecondSquaredTunable.get()
            ));
            hardstopSlowdownMeters = calculateHardstopSlowdownMeters(maxVelocityMetersPerSecondTunable.get());
            robotMechanism.elevator.updateHardstopSlowdownPosition();
        }
    }

    @Override
    public void periodicAfterCommands() {
        // Update current limit
        if (operatorDashboard.manualElevator.get() || goal == Goal.ZERO_ELEVATOR) {
            if (!manualCurrentLimitApplied) {
                io.setManualCurrentLimit(true);
                manualCurrentLimitApplied = true;
            }
        } else {
            if (manualCurrentLimitApplied) {
                io.setManualCurrentLimit(false);
                manualCurrentLimitApplied = false;
            }
        }

        // Handle goal
        Logger.recordOutput("Elevator/Goal", goal);
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Elevator/ClosedLoop", false);
            io.setOpenLoop(0);
            previousStateMeters = null;
        } else if (operatorDashboard.manualElevator.get()) {
            Logger.recordOutput("Elevator/ClosedLoop", false);
            io.setOpenLoop(manualVoltage);
            previousStateMeters = null;
        } else if (goal.setpointMeters != null) {
            double positionMeters = getPositionMeters();
            double velocityMetersPerSec = getVelocityMetersPerSec();

            double setpointMeters = goal.setpointMeters.getAsDouble();
            if (goal.adjustForScoring) {
                setpointMeters += calculatePositionOffsetForScoring();
            }
            setpointMeters = MathUtil.clamp(setpointMeters, 0, maxHeightMeters);

            double offsetMeters = operatorDashboard.elevatorOffsetMeters.get();
            if (offsetMeters != 0.0) {
                setpointMeters += offsetMeters; // Offset should override clamping
                offsetSetAlert.set(true);
            } else {
                offsetSetAlert.set(false);
            }

            boolean usingGentleVelocity = (velocityMetersPerSec < 0 || setpointMeters + 0.1 < positionMeters) // If we are going down
                    // If we are below the hardstop slowdown zone
                    && positionMeters < hardstopSlowdownMeters;
            // Only actually use the gentle profile if we are close enough to the max velocity to avoid jumping directly to max velocity
            boolean usingGentleProfile = usingGentleVelocity && Math.abs(velocityMetersPerSec) < gentleMaxVelocityMetersPerSecond + 0.4;

            if (goal == Goal.STOW && operatorDashboard.coralStuckInRobotMode.get()) {
                // Override stow setpoint if coral is stuck in the robot
                setpointMeters = 1.1;
                // Use gentle so we don't slam coral into one of the crossbars
                usingGentleProfile = true;
            }

            var profile = usingGentleProfile
                    ? profileGentleVelocity
                    : profileFullVelocity;
            // If not using the gentle profile, set the setpoint to the hardstop with the gentle max velocity
            TrapezoidProfile.State setpointState = usingGentleVelocity && !usingGentleProfile
                    ? new TrapezoidProfile.State(hardstopMeters, gentleMaxVelocityMetersPerSecond)
                    : new TrapezoidProfile.State(setpointMeters, 0);

            // Sometimes the profile outruns the elevator, so failsafe if it does
            var usingRealStateAsCurrent = operatorDashboard.useRealElevatorState.get();
            if (usingRealStateAsCurrent) {
                // Turn the toggle off instantly so it's like a button
                // We only want to use the real state for one cycle anyways
                operatorDashboard.useRealElevatorState.set(false);
            }
            var currentState = previousStateMeters == null || usingRealStateAsCurrent
                    ? new TrapezoidProfile.State(positionMeters, velocityMetersPerSec)
                    : previousStateMeters;

            previousStateMeters = profile.calculate(0.02, currentState, setpointState);

            var setpointPositionRad = metersToRad(previousStateMeters.position);
            var setpointVelocityRadPerSec = metersToRad(previousStateMeters.velocity);

            io.setClosedLoop(setpointPositionRad, setpointVelocityRadPerSec);

            Logger.recordOutput("Elevator/ClosedLoop", true);
            Logger.recordOutput("Elevator/UsingGentleVelocity", usingGentleVelocity);
            Logger.recordOutput("Elevator/UsingGentleProfile", usingGentleProfile);
            Logger.recordOutput("Elevator/UsingRealStateAsCurrent", usingRealStateAsCurrent);

            Logger.recordOutput("Elevator/Setpoint/GoalPositionMeters", setpointMeters);
            Logger.recordOutput("Elevator/Setpoint/PositionMeters", previousStateMeters.position);
            Logger.recordOutput("Elevator/Setpoint/VelocityMetersPerSec", previousStateMeters.velocity);
        } else {
            Logger.recordOutput("Elevator/ClosedLoop", false);
            previousStateMeters = null;
        }

        // Check limit switch and zero if needed
        if (operatorDashboard.forceZeroElevator.get()) {
            io.setEncoder(0);
            hasZeroed = true;
            // Turn off the toggle instantly so it's like a button
            operatorDashboard.forceZeroElevator.set(false);
        }
        notZeroedAlert.set(!hasZeroed);
    }

    public Command setGoal(Supplier<Goal> goal) {
        return runOnce(() -> this.goal = goal.get());
    }

    @AutoLogOutput(key = "Elevator/AtGoal")
    private boolean atGoal() {
        // if goal.setpointMeters is null, will be false and won't crash
        return goal.setpointMeters != null
                && Math.abs(goal.setpointMeters.getAsDouble() - getPositionMeters()) <= setpointPositionToleranceMeters
                && Math.abs(getVelocityMetersPerSec()) <= setpointVelocityToleranceMetersPerSec;
    }

    public Command waitUntilAtGoal() {
        return waitUntil(this::atGoal);
    }

    public Command setGoalAndWaitUntilAtGoal(Supplier<Goal> goal) {
        return runOnceAndWaitUntil(() -> this.goal = goal.get(), this::atGoal);
    }

    @AutoLogOutput(key = "Elevator/Measurement/PositionMeters")
    public double getPositionMeters() {
        return radToMeters(inputs.leaderPositionRad);
    }

    @AutoLogOutput(key = "Elevator/Measurement/VelocityMetersPerSec")
    public double getVelocityMetersPerSec() {
        return radToMeters(inputs.leaderVelocityRadPerSec);
    }

    public Command setDistanceFromScoringPositionContinuous(DoubleSupplier distanceFromScoringPositionMeters) {
        // Don't require subsystem - meant to run in background
        return Commands.runEnd(
                () -> this.distanceFromScoringPositionMeters = distanceFromScoringPositionMeters.getAsDouble(),
                () -> this.distanceFromScoringPositionMeters = 0.0
        );
    }

    private double calculatePositionOffsetForScoring() {
        // Safeguard - this shouldn't happen due to when we set the distance but you never know
        if (distanceFromScoringPositionMeters > ReefAlign.alignLinearToleranceMeters) {
            return MathUtil.clamp(distanceFromScoringPositionMeters, 0, 0.5) * positionOffsetPerMeterOfDistance;
        } else {
            return 0.0;
        }
    }

    public double getDriveConstraintScalar() {
        double elevatorSetpoint = goal.setpointMeters != null
                ? goal.setpointMeters.getAsDouble()
                : 0;
        double elevatorPosition = Math.max(getPositionMeters(), elevatorSetpoint);
        return MathUtil.interpolate(
                1,
                DriveConstants.constraintScalarWhenElevatorAtMaxHeightDriver,
                elevatorPosition / maxHeightMeters
        );
    }

    public Command zeroCoral() {
        return CommandsExt.eagerSequence(
                setGoal(() -> Goal.ZERO_CORAL),
                startEndWaitUntil(
                        () -> io.setOpenLoop(-0.7),
                        () -> io.setOpenLoop(0),
                        () -> getPositionMeters() < 0.01
                ),
                Commands.idle()
        );
    }

    public Command zeroElevator() {
        var elevatorInitialPosition = new Object() {
            double val = 0.0;
        };
        return CommandsExt.eagerSequence(
                setGoal(() -> Goal.ZERO_ELEVATOR),
                runOnce(() -> {
                    operatorDashboard.zeroElevatorSequence.set(false);
                    io.setOpenLoop(-0.5);
                }),
                Commands.waitSeconds(0.2),
                Commands.waitUntil(() -> getVelocityMetersPerSec() < 0.02),
                runOnce(() -> {
                    elevatorInitialPosition.val = getPositionMeters();
                    io.setOpenLoop(0.0);
                }),
                Commands.waitSeconds(1),
                runOnce(() -> {
                    if (Math.abs(getPositionMeters() - elevatorInitialPosition.val) < 0.02) {
                        io.setEncoder(0);
                    }
                })
        );
    }

    public Command setManualVoltage(double addedVoltage) {
        // Note - doesn't require subsystem to allow other commands that would require elevator to work
        return Commands.startEnd(
                () -> manualVoltage = gains.kG() + addedVoltage,
                () -> manualVoltage = gains.kG()
        );
    }
}
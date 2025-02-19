package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMechanism;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.util.characterization.FeedforwardCharacterization;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.OperatorDashboard.*;
import static frc.robot.RobotMechanism.middleOfRobot;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.elevator.ElevatorTuning.*;

public class Elevator extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotState.get().getMechanism();

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(null),
        STOW(() -> 0),
        SCORE_L1(scoreL1GoalSetpoint::get),
        SCORE_L2(scoreL2GoalSetpoint::get),
        SCORE_L3(scoreL3GoalSetpoint::get),
        SCORE_L4(scoreL4GoalSetpoint::get),
        DESCORE_L2(descoreL2GoalSetpoint::get),
        DESCORE_L3(descoreL3GoalSetpoint::get);

        /** Should be constant for every loop cycle */
        public final DoubleSupplier setpointMeters;
    }

    @Getter
    private Goal goal = Goal.STOW;

    @AutoLogOutput(key = "Elevator/HasZeroed")
    private boolean hasZeroed = false;
    private boolean autoStop = false;
    private final Timer autoStopTimer = new Timer();
    private boolean prevEmergencyStopped = false;

    private final ElevatorIO io = createIO();
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    /** NOTE: UNITS IN METERS! */
    private TrapezoidProfile profileFullVelocity = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    maxVelocityMetersPerSecond,
                    maxAccelerationMetersPerSecondSquared
            )
    );
    private TrapezoidProfile profileGentleVelocity = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    gentleMaxVelocityMetersPerSecond,
                    maxAccelerationMetersPerSecondSquared
            )
    );
    private TrapezoidProfile.State previousStateMeters = new TrapezoidProfile.State();

    public final SysIdRoutine sysId;

    private final Alert emergencyStoppedAlert = new Alert("Elevator is emergency stopped.", Alert.AlertType.kError);
    private final Alert notZeroedAlert = new Alert("Elevator is not zeroed.", Alert.AlertType.kWarning);
    private final Alert leaderDisconnectedAlert = new Alert("Elevator leader motor is disconnected.", Alert.AlertType.kError);
    private final Alert followerDisconnectedAlert = new Alert("Elevator follower motor is disconnected.", Alert.AlertType.kError);

    private static Elevator instance;

    public static Elevator get() {
        if (instance == null)
            synchronized (Elevator.class) {
                instance = new Elevator();
            }

        return instance;
    }

    private Elevator() {
        sysId = Util.sysIdRoutine(
                "Elevator",
                (voltage) -> io.setOpenLoop(voltage.in(Volts)),
                () -> goal = Goal.CHARACTERIZATION,
                this,
                Volts.per(Second).of(0.2),
                Volts.of(3),
                Seconds.of(20)
        );
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);

        leaderDisconnectedAlert.set(!inputs.leaderConnected);
        followerDisconnectedAlert.set(!inputs.followerConnected);

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

        boolean emergencyStopped = elevatorEStop.get() || autoStop;
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

        var endEffectorX = middleOfRobot - Units.inchesToMeters(10);
        var endEffectorY = Units.inchesToMeters(4.85) + getPositionMeters();
        robotMechanism.endEffector.root.setPosition(endEffectorX, endEffectorY);
        robotMechanism.endEffector.beamBreakRoot.setPosition(endEffectorX - Units.inchesToMeters(1), endEffectorY + Units.inchesToMeters(5.25));
        robotMechanism.endEffector.topRollersRoot.setPosition(endEffectorX - Units.inchesToMeters(3), endEffectorY + Units.inchesToMeters(10));
    }

    @Override
    public void periodicAfterCommands() {
        if (coastOverride.hasChanged(hashCode())) {
            io.setBrakeMode(!coastOverride.get());
        }

        gainsTunable.ifChanged(hashCode(), io::setPIDF);

        if (maxVelocityMetersPerSecondTunable.hasChanged(hashCode())
                || maxAccelerationMetersPerSecondSquaredTunable.hasChanged(hashCode())
        ) {
            profileFullVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    maxVelocityMetersPerSecondTunable.get(),
                    maxAccelerationMetersPerSecondSquaredTunable.get()
            ));
            profileGentleVelocity = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                    gentleMaxVelocityMetersPerSecond,
                    maxAccelerationMetersPerSecondSquaredTunable.get()
            ));
            hardstopSlowdownMeters = calculateHardstopSlowdownMeters(maxVelocityMetersPerSecond);
        }

        // Goal control
        Logger.recordOutput("Elevator/Goal", goal);
        if (goal.setpointMeters != null) {
            var positionMeters = getPositionMeters();
            var velocityMetersPerSec = getVelocityMetersPerSec();
            var setpointMeters = goal.setpointMeters.getAsDouble();

            boolean usingGentleProfile = inputs.leaderVelocityRadPerSec < 0.1 // If we are going down
                    // If we are below the hardstop slowdown zone
                    && positionMeters < hardstopSlowdownMeters;
            var profile = usingGentleProfile
                    ? profileGentleVelocity
                    : profileFullVelocity;

            // Sometimes the profile outruns the elevator, so failsafe if it does
            var usingRealStateAsCurrent = useRealElevatorState.get();
            var currentState = usingRealStateAsCurrent
                    ? new TrapezoidProfile.State(positionMeters, velocityMetersPerSec)
                    : previousStateMeters;
            if (usingRealStateAsCurrent) {
                // Turn it off instantly
                useRealElevatorState.set(false);
            }

            previousStateMeters = profile.calculate(
                    0.02,
                    currentState,
                    new TrapezoidProfile.State(setpointMeters, 0)
            );

            var setpointPositionRad = metersToRad(previousStateMeters.position);
            var setpointVelocityRadPerSec = metersToRad(previousStateMeters.velocity);

            io.setClosedLoop(setpointPositionRad, setpointVelocityRadPerSec);

            Logger.recordOutput("Elevator/ClosedLoop", true);
            Logger.recordOutput("Elevator/UsingGentleProfile", usingGentleProfile);
            Logger.recordOutput("Elevator/UsingRealStateAsCurrent", usingRealStateAsCurrent);

            Logger.recordOutput("Elevator/Setpoint/GoalPositionMeters", setpointMeters);
            Logger.recordOutput("Elevator/Setpoint/PositionMeters", previousStateMeters.position);
            Logger.recordOutput("Elevator/Setpoint/VelocityMetersPerSec", previousStateMeters.velocity);
        } else {
            Logger.recordOutput("Elevator/ClosedLoop", false);
        }

        // Check limit switch and zero if needed
        if (!hasZeroed && inputs.limitSwitchTriggered) {
            io.setEncoder(0);
            hasZeroed = true;
        }
        notZeroedAlert.set(!hasZeroed);
    }

    public Command setGoal(Supplier<Goal> goal) {
        return runOnce(() -> this.goal = goal.get());
    }

    @AutoLogOutput(key = "Elevator/AtGoal")
    private boolean atGoal() {
        // if goal.setpointMeters is null, will be false and won't crash
        return goal.setpointMeters != null && Math.abs(goal.setpointMeters.getAsDouble() - getPositionMeters()) <= setpointToleranceMeters;
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
//        var avgPositionRad = (inputs.leaderPositionRad + inputs.followerPositionRad) / 2.0;
//        return radToMeters(avgPositionRad);
    }

    @AutoLogOutput(key = "Elevator/Measurement/VelocityMetersPerSec")
    public double getVelocityMetersPerSec() {
        return radToMeters(inputs.leaderVelocityRadPerSec);
//        var avgVelocityRadPerSec = (inputs.leaderVelocityRadPerSec + inputs.followerVelocityRadPerSec) / 2.0;
//        return radToMeters(avgVelocityRadPerSec);
    }

    public Command feedforwardCharacterization() {
        return setGoal(() -> Goal.CHARACTERIZATION)
                .andThen(new FeedforwardCharacterization(
                        io::setOpenLoop,
                        () -> new double[]{inputs.leaderVelocityRadPerSec},
                        1,
                        this
                ));
    }
}
package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
import static frc.robot.OperatorDashboard.coastOverride;
import static frc.robot.RobotMechanism.middleOfRobot;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotState.get().getMechanism();

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(null),
        STOW(() -> 0),
        LOW_TESTING_ONLY(() -> 0.5),
        SCORE_L1(() -> 1.69 - Units.inchesToMeters(54)),
        SCORE_L2(() -> 1.69 - Units.inchesToMeters(40.125)),
        SCORE_L3(() -> 1.69 - Units.inchesToMeters(24.375)),
        SCORE_L4(() -> 1.69),
        DESCORE_L2(() -> 0),
        DESCORE_L3(() -> 0);

        /** Should be constant for every loop cycle */
        public final DoubleSupplier setpointMeters;
    }

    @Getter
    private Goal goal = Goal.STOW;

    @AutoLogOutput(key = "Elevator/HasZeroed")
    private boolean hasZeroed = false;

    private static final ElevatorIO io = ElevatorConstants.io;
    private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    /** NOTE: UNITS IN METERS! */
    private final TrapezoidProfile profileFullVelocity = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    maxVelocityMetersPerSecond,
                    maxAccelerationMetersPerSecondSquared
            )
    );
    private final TrapezoidProfile profileGentleVelocity = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    gentleMaxVelocityMetersPerSecond,
                    maxAccelerationMetersPerSecondSquared
            )
    );
    private TrapezoidProfile.State previousStateMeters = new TrapezoidProfile.State();
    private boolean usingRealStateAsCurrent = false;

    public final SysIdRoutine sysId;

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

        // TODO: connected and has zeroed alerts

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

        gainsTunable.ifChanged(io::setPIDF);

        Logger.recordOutput("Elevator/Goal", goal);
        if (goal.setpointMeters != null) {
            var setpointMeters = goal.setpointMeters.getAsDouble();

            boolean usingGentleProfile = inputs.leaderVelocityRadPerSec < 0.1 // If we are going down
                    // If we are below the hardstop slowdown zone
                    && getPositionMeters() < hardstopSlowdownMeters;
            var profile = usingGentleProfile
                    ? profileGentleVelocity
                    : profileFullVelocity;

            // Sometimes the profile outruns the elevator, so failsafe if it does
            usingRealStateAsCurrent = usingRealStateAsCurrent
                    // Stop using the real state as the current state once we get close enough to setpoint
                    // This way, it doesn't go back and forth and is "sticky"
                    ? !(Math.abs(getPositionMeters() - previousStateMeters.position) < 0.05)
                    : (
                    Math.abs(getPositionMeters() - previousStateMeters.position) > 0.25 // If we are too far away from setpoint state
                            && getVelocityMetersPerSec() < 0.1 // If we have stopped
            );
            var currentState = usingRealStateAsCurrent
                    ? new TrapezoidProfile.State(getPositionMeters(), getVelocityMetersPerSec())
                    : previousStateMeters;

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

        if (!hasZeroed && inputs.limitSwitchTriggered) {
            io.setEncoder(0);
            hasZeroed = true;
        }
    }

    public Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    @AutoLogOutput(key = "Elevator/AtGoal")
    private boolean atGoal() {
        // if goal.setpointMeters is null, will be false and won't crash
        return goal.setpointMeters != null && Math.abs(metersToRad(goal.setpointMeters.getAsDouble()) - inputs.leaderPositionRad) <= setpointToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return waitUntil(this::atGoal);
    }

    public Command setGoalAndWaitUntilAtGoal(Supplier<Goal> goal) {
        return runOnceAndWaitUntil(() -> this.goal = goal.get(), this::atGoal);
    }

    @AutoLogOutput(key = "Elevator/Measurement/PositionMeters")
    public double getPositionMeters() {
        var avgPositionRad = (inputs.leaderPositionRad + inputs.followerPositionRad) / 2.0;
        return radToMeters(avgPositionRad);
    }

    @AutoLogOutput(key = "Elevator/Measurement/VelocityMetersPerSec")
    public double getVelocityMetersPerSec() {
        var avgVelocityRadPerSec = (inputs.leaderVelocityRadPerSec + inputs.followerVelocityRadPerSec) / 2.0;
        return radToMeters(avgVelocityRadPerSec);
    }

    public Command feedforwardCharacterization() {
        return setGoal(Goal.CHARACTERIZATION)
                .andThen(new FeedforwardCharacterization(
                        io::setOpenLoop,
                        () -> new double[]{inputs.leaderVelocityRadPerSec},
                        1,
                        this
                ));
    }

    public Command runOpenLoop() {
        return setGoal(Goal.CHARACTERIZATION)
                .andThen(run(() -> io.setOpenLoop(1)))
                .finallyDo(() -> io.setOpenLoop(0));
    }
}
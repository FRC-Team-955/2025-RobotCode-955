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

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotMechanism.middleOfRobot;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotState.get().getMechanism();

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(null),
        STOW(() -> 0),
        SCORE_L1(() -> 0),
        SCORE_L2(() -> 0),
        SCORE_L3(() -> 0),
        SCORE_L4(() -> 1.69),
        DESCORE_L2(() -> 0),
        DESCORE_L3(() -> 0);

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointMeters;
    }

    @Getter
    private Goal goal = Goal.STOW;

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
    private TrapezoidProfile.State previousState = new TrapezoidProfile.State();

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
                this
        );
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);

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
        ////////////// PIVOT //////////////
        Logger.recordOutput("Elevator/Goal", goal);
        if (goal.setpointMeters != null) {
            var setpointMeters = goal.setpointMeters.getAsDouble();

            var shouldUseGentleProfile = false;
            //noinspection ConstantValue
            var profile = shouldUseGentleProfile
                    ? profileGentleVelocity
                    : profileFullVelocity;
            previousState = profile.calculate(
                    0.02,
                    new TrapezoidProfile.State(getPositionMeters(), getVelocityMetersPerSec()),
//                    previousState,
                    new TrapezoidProfile.State(setpointMeters, 0)
            );

            var setpointPositionRad = metersToRad(previousState.position);
            var setpointVelocityRadPerSec = metersToRad(previousState.velocity);
            io.setClosedLoop(setpointPositionRad, setpointVelocityRadPerSec);

            Logger.recordOutput("Elevator/ClosedLoop", true);
            Logger.recordOutput("Elevator/UsingGentleProfile", shouldUseGentleProfile);

            Logger.recordOutput("Elevator/Setpoint/PositionMeters", previousState.position);
            Logger.recordOutput("Elevator/Setpoint/VelocityMetersPerSec", previousState.velocity);
        } else {
            Logger.recordOutput("Elevator/ClosedLoop", false);
        }
    }

    public Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    @AutoLogOutput(key = "Elevator/AtGoal")
    private boolean atGoal() {
        // if goal.setpointMeters is null, will be false and won't crash
        return goal.setpointMeters != null && Math.abs(metersToRad(goal.setpointMeters.getAsDouble()) - inputs.positionRad) <= setpointToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return waitUntil(this::atGoal);
    }

    public Command setGoalAndWaitUntilAtGoal(Goal goal) {
        return runOnceAndWaitUntil(() -> this.goal = goal, this::atGoal);
    }

    @AutoLogOutput(key = "Elevator/Measurement/PositionMeters")
    public double getPositionMeters() {
        return radToMeters(inputs.positionRad);
    }

    @AutoLogOutput(key = "Elevator/Measurement/VelocityMetersPerSec")
    public double getVelocityMetersPerSec() {
        return radToMeters(inputs.velocityRadPerSec);
    }

    public Command gravityCharacterization() {
        return setGoal(Goal.CHARACTERIZATION)
                .andThen(new FeedforwardCharacterization(
                        io::setOpenLoop,
                        () -> inputs.velocityRadPerSec,
                        this
                ));
    }
}
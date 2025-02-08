package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotState;
import frc.robot.Util;
import frc.robot.util.characterization.FeedforwardCharacterization;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();

    private final LoggedMechanismRoot2d stage1Root = robotState.getMechanism2d().getRoot("elevatorStage1", 0, 0);
    private final LoggedMechanismRoot2d stage2Root = robotState.getMechanism2d().getRoot("elevatorStage2", 0, 0);
    private final LoggedMechanismRoot2d stage3Root = robotState.getMechanism2d().getRoot("elevatorStage3", 0, 0);
    private final LoggedMechanismRoot2d endEffectorRoot = robotState.getMechanism2d().getRoot("endEffector", 0, 0);
    private final LoggedMechanismLigament2d endEffectorLigament = endEffectorRoot.append(new LoggedMechanismLigament2d(
            "endEffector",
            Units.inchesToMeters(10),
            90,
            10,
            new Color8Bit(Color.kOrange)
    ));

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(null),
        STOW(() -> 0),
        SCORE_L1(() -> 0),
        SCORE_L2(() -> 0),
        SCORE_L3(() -> 0),
        SCORE_L4(() -> 1.6),
        DESCORE_L2(() -> 0),
        DESCORE_L3(() -> 0);

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointMeters;
    }

    private static final ElevatorIO io = ElevatorConstants.io;
    private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    @Getter
    private Goal goal = Goal.STOW;

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

        var baseRoot = robotState.getMechanism2d()
                .getRoot("elevatorBase", 0.5 + Units.inchesToMeters(7) - 0.06, Units.inchesToMeters(1.85));
        baseRoot.append(new LoggedMechanismLigament2d(
                "base",
                Units.inchesToMeters(33.2),
                90,
                13,
                new Color8Bit(new Color(0.2, 0.2, 0.2))
        ));
        stage1Root.append(new LoggedMechanismLigament2d(
                "stage1",
                Units.inchesToMeters(32.5),
                90,
                12,
                new Color8Bit(new Color(0.3, 0.3, 0.3))
        ));
        stage2Root.append(new LoggedMechanismLigament2d(
                "stage2",
                Units.inchesToMeters(32),
                90,
                11,
                new Color8Bit(new Color(0.4, 0.4, 0.4))
        ));
        stage3Root.append(new LoggedMechanismLigament2d(
                "stage3",
                Units.inchesToMeters(7),
                90,
                10,
                new Color8Bit(new Color(0.5, 0.5, 0.5))
        ));
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);

        stage1Root.setPosition(0.5 + Units.inchesToMeters(7) - 0.04, Units.inchesToMeters(2.85) + getPositionMeters() / 3);
        stage2Root.setPosition(0.5 + Units.inchesToMeters(7) - 0.02, Units.inchesToMeters(3.85) + getPositionMeters() / 3 * 2);
        stage3Root.setPosition(0.5 + Units.inchesToMeters(7), Units.inchesToMeters(4.85) + getPositionMeters());
        endEffectorRoot.setPosition(0.5 + Units.inchesToMeters(10), Units.inchesToMeters(4.85) + getPositionMeters());
        endEffectorLigament.setAngle(MathUtil.clamp(
                // After 5 inches, interpolate to 40 degrees finishing at 7.25 inches
                90 - (40 / Units.inchesToMeters(2.25) * (getPositionMeters() - Units.inchesToMeters(5))),
                50, 90
        ));
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
                    previousState, // TODO: should we measure the current state instead of assuming previous is where we are currently at?
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
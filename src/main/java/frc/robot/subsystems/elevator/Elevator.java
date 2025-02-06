package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Util;
import frc.robot.util.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemBaseExt {
    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(null),
        STOW(() -> 0),
        SCORE_L1(() -> 0),
        SCORE_L2(() -> 0),
        SCORE_L3(() -> 0),
        SCORE_L4(() -> 0),
        DESCORE_L2(() -> 0),
        DESCORE_L3(() -> 0);

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointMeters;
    }

    private static final ElevatorIO io = ElevatorConstants.io;
    private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    @Getter
    private Goal goal = Goal.STOW;

    private static Elevator instance;

    public static Elevator get() {
        if (instance == null)
            synchronized (Elevator.class) {
                instance = new Elevator();
            }

        return instance;
    }

    private Elevator() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Elevator", inputs);
    }

    @Override
    public void periodicAfterCommands() {
        ////////////// PIVOT //////////////
        Logger.recordOutput("Elevator/Goal", goal);
        if (goal.setpointMeters != null) {
            var setpointRad = metersToRad(goal.setpointMeters.getAsDouble());
            var maxVelocityRadPerSec = getPositionMeters() > hardStopHeightMeters
                    ? metersToRad(maxVelocityAboveHardStopMetersPerSecond)
                    : metersToRad(maxVelocityBelowHardStopMetersPerSecond);
            io.setPosition(setpointRad, maxVelocityRadPerSec);
            Logger.recordOutput("Elevator/ClosedLoop", true);
            Logger.recordOutput("Elevator/SetpointRad", setpointRad);
            Logger.recordOutput("Elevator/MaxVelocityRadPerSec", maxVelocityRadPerSec);
        } else {
            setpointRad = null;
            Logger.recordOutput("Elevator/ClosedLoop", false);
        }
    }

    public Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    @AutoLogOutput(key = "Elevator/AtGoal")
    private boolean atGoal() {
        // if goal.setpointMeters is null, will be false and won't crash
        return goal.setpointMeters != null && Math.abs(metersToRad(goal.setpointMeters.get()) - inputs.positionRad) <= setpointToleranceRad;
    }

    public Command waitUntilAtGoal() {
        return waitUntil(this::atGoal);
    }

    public Command setGoalAndWaitUntilAtGoal(Goal goal) {
        return runOnceAndWaitUntil(() -> this.goal = goal, this::atGoal);
    }

    @AutoLogOutput(key = "Elevator/PositionMeters")
    public double getPositionMeters() {
        return radToMeters(inputs.positionRad);
    }

    @AutoLogOutput(key = "Elevator/VelocityMetersPerSec")
    public double getVelocityMetersPerSec() {
        return radToMeters(inputs.velocityRadPerSec);
    }
}
package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        private final DoubleSupplier setpointMeters;
    }

    private static final ElevatorIO io = ElevatorConstants.io;
    private static final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    @Getter
    private Goal goal = Goal.STOW;
    private Double setpointRad;

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
            setpointRad = goal.setpointMeters.getAsDouble();
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
        return Math.abs(setpointRad - inputs.positionRad) <= setpointToleranceRad;
    }

    public Command waitUntilAtGoal() {
        var cmd = Commands.waitUntil(this::atGoal);
        cmd.addRequirements(this);
        return cmd;
    }

    @AutoLogOutput(key = "Elevator/PositionMeters")
    public double getPositionMeters() {
        return radToMeters(inputs.positionRad);
    }
}
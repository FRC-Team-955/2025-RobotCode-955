package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.coralintake.CoralIntakeConstants.pivotConfig;

public class CoralIntake extends SubsystemBaseExt {
    @RequiredArgsConstructor
    public enum PivotGoal {
        CHARACTERIZATION(null),
        STOW(() -> 0),
        INTAKE(() -> 1);

        /** Should be constant for every loop cycle */
        private final DoubleSupplier setpointRad;
    }

    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        INTAKE(() -> 1),
        EJECT(() -> -1);

        private final DoubleSupplier setpointRadPerSec;
    }

    private static final PivotIO pivotIO = new PivotIO();
    private static final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

    @Getter
    private PivotGoal pivotGoal = PivotGoal.STOW;

    private static final RollersIO rollersIO = new RollersIO();
    private static final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    @Getter
    private RollersGoal rollersGoal = RollersGoal.IDLE;

    private static CoralIntake instance;

    public static CoralIntake get() {
        if (instance == null)
            synchronized (CoralIntake.class) {
                instance = new CoralIntake();
            }

        return instance;
    }

    private CoralIntake() {
    }

    @Override
    public void periodicBeforeCommands() {
        pivotIO.updateInputs(pivotInputs);
        Logger.processInputs("Inputs/CoralIntake/Pivot", pivotInputs);

        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/CoralIntake/Rollers", rollersInputs);
    }

    @Override
    public void periodicAfterCommands() {
        ////////////// PIVOT //////////////
        Logger.recordOutput("CoralIntake/Pivot/Goal", pivotGoal);
        if (pivotGoal.setpointRad != null) {
            var pivotSetpointRad = pivotGoal.setpointRad.getAsDouble();
            pivotIO.setPosition(pivotSetpointRad);
            Logger.recordOutput("CoralIntake/Pivot/ClosedLoop", true);
            Logger.recordOutput("CoralIntake/Pivot/SetpointRad", pivotSetpointRad);
        } else {
            Logger.recordOutput("CoralIntake/Pivot/ClosedLoop", false);
        }

        ////////////// ROLLERS //////////////
        Logger.recordOutput("CoralIntake/Rollers/Goal", rollersGoal);
        if (rollersGoal.setpointRadPerSec != null) {
            var rollersSetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
            rollersIO.setVelocity(rollersSetpointRadPerSec);
            Logger.recordOutput("CoralIntake/Rollers/ClosedLoop", true);
            Logger.recordOutput("CoralIntake/Rollers/SetpointRadPerSec", rollersSetpointRadPerSec);
        } else {
            Logger.recordOutput("CoralIntake/Rollers/ClosedLoop", false);
        }
    }

    public Command setGoals(PivotGoal pivotGoal, RollersGoal rollersGoal) {
        return runOnce(() -> {
            this.pivotGoal = pivotGoal;
            this.rollersGoal = rollersGoal;
        });
    }

    private boolean atPivotGoal() {
        // if pivotGoal.setpointRad is null, will be false and won't crash
        return pivotGoal.setpointRad != null && Math.abs(pivotGoal.setpointRad.getAsDouble() - pivotInputs.positionRad) <= pivotConfig.setpointToleranceRad();
    }

    public Command waitUntilAtPivotGoal() {
        return waitUntil(this::atPivotGoal);
    }

    public Command setGoalsAndWaitUntilAtPivotGoal(PivotGoal pivotGoal, RollersGoal rollersGoal) {
        return runOnceAndWaitUntil(
                () -> {
                    this.pivotGoal = pivotGoal;
                    this.rollersGoal = rollersGoal;
                },
                this::atPivotGoal
        );
    }
}
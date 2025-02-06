package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Indexer extends SubsystemBaseExt {
    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        INDEX(() -> 1),
        EJECT(() -> -1);

        private final DoubleSupplier setpointRadPerSec;
    }

    private static final RollersIO rollersIO = IndexerConstants.rollersIO;
    private static final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    @Getter
    private RollersGoal rollersGoal = RollersGoal.IDLE;

    private static Indexer instance;

    public static Indexer get() {
        if (instance == null)
            synchronized (Indexer.class) {
                instance = new Indexer();
            }

        return instance;
    }

    private Indexer() {
    }

    @Override
    public void periodicBeforeCommands() {
        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/Indexer/Rollers", rollersInputs);
    }

    @Override
    public void periodicAfterCommands() {
        ////////////// ROLLERS //////////////
        Logger.recordOutput("Indexer/Rollers/Goal", rollersGoal);
        if (rollersGoal.setpointRadPerSec != null) {
            var rollersSetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
            rollersIO.setVelocity(rollersSetpointRadPerSec);
            Logger.recordOutput("Indexer/Rollers/ClosedLoop", true);
            Logger.recordOutput("Indexer/Rollers/SetpointRadPerSec", rollersSetpointRadPerSec);
        } else {
            Logger.recordOutput("Indexer/Rollers/ClosedLoop", false);
        }
    }

    public Command setGoal(RollersGoal rollersGoal) {
        return runOnce(() -> this.rollersGoal = rollersGoal);
    }
}
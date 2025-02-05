package frc.robot.subsystems.endeffector;

import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;
import lombok.RequiredArgsConstructor;

public class EndEffector extends SubsystemBaseExt {
    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(() -> null),
        IDLE(() -> 0),
        FORWARD(() -> 1);

        private final Supplier<Double> setpointRadPerSec;
    }

    private static final RollersIO rollersIO;
    private static final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    private RollersGoal rollersGoal = RollersGoal.IDLE;
    private Double rollersSetpointRadPerSec;

    private static EndEffector instance;

    public static EndEffector get() {
        if (instance == null)
            synchronized (EndEffector.class) {
                instance = new EndEffector();
            }

        return instance;
    }

    @Override
    public void periodicBeforeCommands() {
        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/EndEffector/Rollers", rollersInputs);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("EndEffector/Rollers/Goal", rollersGoal);
        rollersSetpointRadPerSec = rollersGoal.setpointRadPerSec.get();

        if (rollersSetpointRadPerSec != null) {
            Logger.recordOutput("EndEffector/Rollers/ClosedLoop", true);
        } else {
            Logger.recordOutput("EndEffector/Rollers/ClosedLoop", false);
        }
    }
}
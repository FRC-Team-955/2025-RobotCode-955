package frc.robot.subsystems.endeffector;

import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;
import lombok.RequiredArgsConstructor;

public class EndEffector extends SubsystemBaseExt {
    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(0), // specially handled in periodic
        IDLE(0),
        FORWARD(1);

        private final double setpointRadPerSec;
    }

    private static final RollersIO rollersIO;
    private static final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    private RollersGoal rollersGoal = RollersGoal.IDLE;
    private double rollersSetpointRadPerSec;

    private static EndEffector instance;

    public static EndEffector get() {
        if (instance == null)
            synchronized (EndEffector.class) {
                instance = new EndEffector();
            }

        return instance;
    }
}
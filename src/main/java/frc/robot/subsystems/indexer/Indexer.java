package frc.robot.subsystems.indexer;

import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;

public class Indexer extends SubsystemBaseExt {
    public enum RollersGoal {
    }

    private static final RollersIO rollersIO;
    private static final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    private RollersGoal rollersGoal = RollersGoal.IDLE;
    private double rollersSetpointRadPerSec;

    private static Indexer instance;

    public static Indexer get() {
        if (instance == null)
            synchronized (Indexer.class) {
                instance = new Indexer();
            }

        return instance;
    }
}
package frc.robot.subsystems.coralintake;

import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;

public class CoralIntake extends SubsystemBaseExt {
    public enum PivotGoal {
    }

    public enum RollersGoal {
    }

    private static final PivotIO pivotIO;
    private static final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

    private PivotGoal pivotGoal = PivotGoal.IDLE;
    private double pivotSetpointRad;

    private static final RollersIO rollersIO;
    private static final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    private RollersGoal rollersGoal = RollersGoal.IDLE;
    private double rollersSetpointRadPerSec;

    private static CoralIntake instance;

    public static CoralIntake get() {
        if (instance == null)
            synchronized (CoralIntake.class) {
                instance = new CoralIntake();
            }

        return instance;
    }
}
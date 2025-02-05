package frc.robot.subsystems.coralintake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;
import lombok.RequiredArgsConstructor;

import static frc.robot.subsystems.coralintake.CoralIntakeConstants.pivotConfig;

public class CoralIntake extends SubsystemBaseExt {
    public enum PivotGoal {
    }

    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(0), // specially handled in periodic
        IDLE(0),
        INTAKE(1),
        EJECT(-1);

        private final double setpointRadPerSec;
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

    public Command setPivotGoal(PivotGoal pivotGoal) {
        return runOnce(() -> this.pivotGoal = pivotGoal);
    }

    public Command setRollersGoal(RollersGoal rollersGoal) {
        return runOnce(() -> this.rollersGoal = rollersGoal);
    }

    public boolean atPivotGoal() {
        return Math.abs(pivotSetpointRad - pivotInputs.positionRad) <= pivotConfig.setpointToleranceRad();
    }
}
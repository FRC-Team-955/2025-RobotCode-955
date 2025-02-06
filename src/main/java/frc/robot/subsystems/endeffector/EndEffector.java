package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class EndEffector extends SubsystemBaseExt {
    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        FORWARD(() -> 1);

        private final DoubleSupplier setpointRadPerSec;
    }

    private static final RollersIO rollersIO = EndEffectorConstants.rollersIO;
    private static final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    @Getter
    private RollersGoal rollersGoal = RollersGoal.IDLE;

    private static EndEffector instance;

    public static EndEffector get() {
        if (instance == null)
            synchronized (EndEffector.class) {
                instance = new EndEffector();
            }

        return instance;
    }

    private EndEffector() {
    }

    @Override
    public void periodicBeforeCommands() {
        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/EndEffector/Rollers", rollersInputs);
    }

    @Override
    public void periodicAfterCommands() {
        ////////////// ROLLERS //////////////
        Logger.recordOutput("EndEffector/Rollers/Goal", rollersGoal);
        if (rollersGoal.setpointRadPerSec != null) {
            var rollersSetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
            rollersIO.setVelocity(rollersSetpointRadPerSec);
            Logger.recordOutput("EndEffector/Rollers/ClosedLoop", true);
            Logger.recordOutput("EndEffector/Rollers/SetpointRadPerSec", rollersSetpointRadPerSec);
        } else {
            Logger.recordOutput("EndEffector/Rollers/ClosedLoop", false);
        }
    }

    public Command setGoal(RollersGoal rollersGoal) {
        return runOnce(() -> this.rollersGoal = rollersGoal);
    }
}
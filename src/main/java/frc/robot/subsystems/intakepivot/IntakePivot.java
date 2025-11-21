package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotMechanism;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.intakepivot.IntakeConstants.intakeSetpointToleranceRad;
import static frc.robot.subsystems.intakepivot.IntakeTuning.moduleIntakeGainsTunable;

public class IntakePivot implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private static final IntakePivotIO intakePivotIO = IntakeConstants.intakePivotIO;
    private static final IntakePivotIOInputsAutoLogged intakeInputs = new IntakePivotIOInputsAutoLogged();

    private Command defaultCommand;

    @RequiredArgsConstructor
    public enum IntakePivotGoal {
        CHARACTERIZATION(null),
        STOW(() -> 1.353),
        INTAKE(() -> 0.12833586);

        private final DoubleSupplier setpointRad;
    }

    private IntakePivotGoal intakePivotGoal = IntakePivotGoal.STOW;

    private static IntakePivot instance;
    public static IntakePivot get() {
        if (instance == null) {
            synchronized (IntakePivot.class) {
                instance = new IntakePivot();
            }
        }
        return instance;
    }

    private IntakePivot() {}

    @Override
    public void periodicBeforeCommands() {
        intakePivotIO.updateInputs(intakeInputs);
        Logger.processInputs("Inputs/Intake/Pivot", intakeInputs);

        moduleIntakeGainsTunable.ifChanged(intakePivotIO::setIntakePIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Intake/Pivot/Goal", intakePivotGoal);
        if (intakePivotGoal.setpointRad != null) {
            double intakeSetpointRad = intakePivotGoal.setpointRad.getAsDouble();
            intakePivotIO.setClosedLoop(intakeSetpointRad);
            Logger.recordOutput("Intake/Pivot/ClosedLoop", true);
            Logger.recordOutput("Intake/Pivot/SetpointRad", intakeSetpointRad);
        } else {
            Logger.recordOutput("Intake/Pivot/ClosedLoop", false);
        }

        if (defaultCommand != null && !defaultCommand.isScheduled()) {
            defaultCommand.schedule();
        }

        robotMechanism.PivotMechanism.update();
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
        if (command != null) {
            command.schedule();
        }
    }

    public double getCurrentAngleRad() {
        return intakeInputs.positionRad;
    }

    public IntakePivotGoal getCurrentGoal() {
        return intakePivotGoal;
    }

    public Command setGoals(IntakePivotGoal intakePivotGoal) {
        return runOnce(() -> this.intakePivotGoal = intakePivotGoal);
    }

    private boolean atIntakePivotGoal() {
        return intakePivotGoal.setpointRad != null &&
                Math.abs(intakePivotGoal.setpointRad.getAsDouble() - intakeInputs.positionRad) <= intakeSetpointToleranceRad;
    }

    public Command waitUntilAtIntakeGoal() {
        return run(() -> {}).until(this::atIntakePivotGoal);
    }

    public Command setGoalsAndWaitUntilAtIntakeGoal(IntakePivotGoal intakePivotGoal) {
        return runOnce(() -> this.intakePivotGoal = intakePivotGoal).andThen(waitUntilAtIntakeGoal());
    }
}

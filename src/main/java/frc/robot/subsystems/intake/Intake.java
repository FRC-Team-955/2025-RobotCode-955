package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMechanism;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConstants.intakeSetpointToleranceRad;
import static frc.robot.subsystems.intake.IntakeTuning.moduleIntakeGainsTunable;

public class Intake extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private static final IntakeIO intakeIO = IntakeConstants.intakeIo;
    private static final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum IntakeGoal {
        CHARACTERIZATION(null),
        STOW(() -> 1.353),
        INTAKE(() -> 0.12833586);

        private final DoubleSupplier setpointRad;
    }

    @Getter
    private IntakeGoal intakeGoal;

    private static Intake instance;

    public static Intake get() {
        if (instance == null)
            synchronized (Intake.class) {
                instance = new Intake();
            }
        return instance;
    }

    private Intake() {
        this.intakeGoal = IntakeGoal.STOW;
    }

    @Override
    public void periodicBeforeCommands() {
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Inputs/Intake/Pivot", intakeInputs);

        moduleIntakeGainsTunable.ifChanged(intakeIO::setPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        moduleIntakeGainsTunable.ifChanged(intakeIO::setPIDF);
        Logger.recordOutput("Intake/Pivot/Goal", intakeGoal);
        if (intakeGoal.setpointRad != null) {
            var intakeSetpointRad = intakeGoal.setpointRad.getAsDouble();
            intakeIO.setClosedLoop(intakeSetpointRad);
            Logger.recordOutput("Intake/Pivot/ClosedLoop", true);
            Logger.recordOutput("Intake/Pivot/SetpointRad", intakeSetpointRad);
        } else {
            Logger.recordOutput("Intake/Pivot/ClosedLoop", false);
        }
    }

    public Command setGoals(IntakeGoal intakeGoal) {
        return runOnce(() -> {
            this.intakeGoal = intakeGoal;
        });
    }

    private boolean atIntakeGoal() {
        return intakeGoal.setpointRad != null && Math.abs(intakeGoal.setpointRad.getAsDouble() - intakeInputs.positionRad) <= intakeSetpointToleranceRad;
    }

    public Command waitUntilAtIntakeGoal() {
        return waitUntil(this::atIntakeGoal);
    }

    public Command setGoalsAndWaitUntilAtIntakeGoal(IntakeGoal intakeGoal) {
        return runOnceAndWaitUntil(
                () -> {
                    this.intakeGoal = intakeGoal;
                },
                this::atIntakeGoal
        );
    }

}

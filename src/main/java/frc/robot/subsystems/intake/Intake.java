package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMechanism;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.intake.IntakeConstants.intakeSetpointToleranceRad;
import static frc.robot.subsystems.intake.IntakeTuning.moduleIntakeGainsTunable;

public class Intake extends SubsystemBase {
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private static final IntakeIO intakeIO = IntakeConstants.intakeIO;
    private static final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    public enum IntakeGoal {
        CHARACTERIZATION(null),
        STOW(() -> 1.353),
        INTAKE(() -> 0.12833586);

        private final DoubleSupplier setpointRad;
        IntakeGoal(DoubleSupplier setpointRad) { this.setpointRad = setpointRad; }
    }

    private IntakeGoal intakeGoal = IntakeGoal.STOW;

    private static Intake instance;
    public static Intake get() {
        if (instance == null) {
            synchronized (Intake.class) {
                instance = new Intake();
            }
        }
        return instance;
    }

    private Intake() {}

    @Override
    public void periodic() {
        // Combine both before/after sections
        intakeIO.updateInputs(intakeInputs);
        Logger.processInputs("Inputs/Intake/Pivot", intakeInputs);

        moduleIntakeGainsTunable.ifChanged(intakeIO::setIntakePIDF);

        Logger.recordOutput("Intake/Pivot/Goal", intakeGoal);
        if (intakeGoal.setpointRad != null) {
            double intakeSetpointRad = intakeGoal.setpointRad.getAsDouble();
            intakeIO.setClosedLoop(intakeSetpointRad);
            Logger.recordOutput("Intake/Pivot/ClosedLoop", true);
            Logger.recordOutput("Intake/Pivot/SetpointRad", intakeSetpointRad);
        } else {
            Logger.recordOutput("Intake/Pivot/ClosedLoop", false);
        }
    }

    public Command setGoals(IntakeGoal intakeGoal) {
        return runOnce(() -> this.intakeGoal = intakeGoal);
    }

    private boolean atIntakeGoal() {
        return intakeGoal.setpointRad != null &&
                Math.abs(intakeGoal.setpointRad.getAsDouble() - intakeInputs.positionRad) <= intakeSetpointToleranceRad;
    }

    public Command waitUntilAtIntakeGoal() {
        return run(() -> {}).until(this::atIntakeGoal);
    }

    public Command setGoalsAndWaitUntilAtIntakeGoal(IntakeGoal intakeGoal) {
        return runOnce(() -> this.intakeGoal = intakeGoal).andThen(waitUntilAtIntakeGoal());
    }
}

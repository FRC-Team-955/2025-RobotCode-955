package frc.robot.subsystems.intakerollers;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.intakepivot.IntakePivot;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
//import static frc.robot.subsystems.intakerollers.IntakeRollersTuning.moduleIntakeRollersPositionGainsTunable;
import static frc.robot.subsystems.intakerollers.IntakeRollersTuning.moduleIntakeRollersVelocityGainsTunable;

public class IntakeRollers implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private final IntakeRollersIO intakeRollersIO = IntakeRollersConstants.intakeRollersIO;
    private final IntakeRollersIOInputsAutoLogged intakeRollersInputs = new IntakeRollersIOInputsAutoLogged();

    private Command defaultCommand;

    @RequiredArgsConstructor
    public enum IntakeRollersGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        INTAKE(() -> Units.rotationsPerMinuteToRadiansPerSecond(-200)),
        EJECT(() -> 1);

        private final DoubleSupplier setpointRadPerSec;
    }

    @Getter
    private IntakeRollersGoal intakeRollersGoal = IntakeRollersGoal.IDLE;

    private static IntakeRollers instance;
    public static IntakeRollers get() {
        if (instance == null) {
            synchronized (IntakeRollers.class) {
                instance = new IntakeRollers();
            }
        }
        return instance;
    }

    @Override
    public void periodicBeforeCommands() {
        intakeRollersIO.updateInputs(intakeRollersInputs);
        Logger.processInputs("Inputs/Intake/Rollers", intakeRollersInputs);

//        moduleIntakeRollersPositionGainsTunable.ifChanged(intakeRollersIO::setPositionPIDF);
        moduleIntakeRollersVelocityGainsTunable.ifChanged(intakeRollersIO::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Intake/Rollers/Goal", intakeRollersGoal);
        if (intakeRollersGoal.setpointRadPerSec != null) {
            var intakeRollersSetpointRadPerSec = intakeRollersGoal.setpointRadPerSec.getAsDouble();
            intakeRollersIO.setVelocity(intakeRollersSetpointRadPerSec);
            Logger.recordOutput("Intake/Rollers/ClosedLoop", true);
            Logger.recordOutput("Intake/Rollers/SetpointRadPerSec", intakeRollersSetpointRadPerSec);
        } else {
            Logger.recordOutput("Intake/Rollers/ClosedLoop", false);
        }

        if (defaultCommand != null && !defaultCommand.isScheduled()) {
            defaultCommand.schedule();
        }
    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
        if (command != null) {
            command.schedule();
        }
    }

    public IntakeRollersGoal getCurrentGoal() {
        return intakeRollersGoal;
    }

    public Command setGoals(IntakeRollersGoal intakeRollersGoal) {
        return runOnce(() -> this.intakeRollersGoal = intakeRollersGoal);
    }
}

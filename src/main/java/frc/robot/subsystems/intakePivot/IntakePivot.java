package frc.robot.subsystems.intakePivot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.lib.commands.CommandsExt.runOnceAndWaitUntil;
import static frc.robot.RobotMechanism.middleOfRobot;
import static frc.robot.subsystems.intakePivot.IntakePivotConstants.*;
import static frc.robot.subsystems.intakePivot.IntakePivotTuning.IntakePivotGainsTunable;

public class IntakePivot implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private Command defaultCommand;
    private static final IntakePivotIO io = createIO();
    private static final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum IntakePivotGoal {
        CHARACTERIZATION(null),
        STOW(() -> 1.353),
        INTAKE(() -> 0.12833586);

        private final DoubleSupplier setpointRad;
    }

    @Getter
    @Setter
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

    private IntakePivot() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/IntakePivot", inputs);
        robotMechanism.intakePivot.root.setPosition(
                middleOfRobot +
                        Units.inchesToMeters(6.5) + intakeLengthMeters * Math.cos(inputs.positionRad),
                Units.inchesToMeters(5.5)
                        + intakeLengthMeters * Math.sin(inputs.positionRad)
        );

        IntakePivotGainsTunable.ifChanged(io::setIntakePIDF);

    }


    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("IntakePivot/Goal", intakePivotGoal);


        if (intakePivotGoal.setpointRad != null) {
            var intakeSetpointRad = intakePivotGoal.setpointRad.getAsDouble();
            io.setClosedLoop(intakeSetpointRad);
            Logger.recordOutput("IntakePivot/ClosedLoop", true);
            Logger.recordOutput("IntakePivot/SetpointRad", intakeSetpointRad);
        } else {
            Logger.recordOutput("IntakePivot/ClosedLoop", false);
        }

        if (defaultCommand != null && !defaultCommand.isScheduled()) {
            defaultCommand.schedule();

        }


    }

    @AutoLogOutput(key = "IntakePivot/PositionRad")
    public double getPositionRad() {
        return inputs.positionRad;

    }

    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
        if (command != null) {
            command.schedule();
        }
    }
//
//    public Command setGoals(IntakePivotGoal intakePivotGoal) {
//        return runOnce(() ->
//                this.intakePivotGoal = intakePivotGoal);
//    }

//    @AutoLogOutput(key = "IntakePivot/CurrentGoal")
//    public IntakePivotGoal getCurrentGoal() {
//        return intakePivotGoal;
//    }

    @AutoLogOutput(key = "IntakePivot/AtGoal")
    private boolean atIntakePivotGoal() {
        return intakePivotGoal.setpointRad != null &&
                Math.abs(intakePivotGoal.setpointRad.getAsDouble() - inputs.positionRad) <= intakeSetpointToleranceRad;
    }

    public Command waitUntilAtIntakeGoal() {
        return Commands.waitUntil(this::atIntakePivotGoal);
    }

    public Command setGoalsAndWaitUntilAtIntakeGoal(IntakePivotGoal intakePivotGoal) {
        return runOnceAndWaitUntil(
                () -> {
                    this.intakePivotGoal = intakePivotGoal;
                },
                this::atIntakePivotGoal
        );
    }
}

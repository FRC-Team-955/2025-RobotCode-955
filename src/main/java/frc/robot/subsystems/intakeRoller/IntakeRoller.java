package frc.robot.subsystems.intakeRoller;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.robot.subsystems.intakeRoller.IntakeRollerConstants.createIO;
import static frc.robot.subsystems.intakeRoller.IntakeRollerTuning.intakeRollerPositionGainsTunable;
import static frc.robot.subsystems.intakeRoller.IntakeRollerTuning.intakeRollerVelocityGainsTunable;


public class IntakeRoller implements Periodic {

    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final static IntakeRollerIO io = createIO();
    private static final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    private Command defaultCommand;

    @RequiredArgsConstructor
    public enum IntakeRollerGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        INTAKE(() -> Units.rotationsPerMinuteToRadiansPerSecond(-200)),
        EJECT(() -> 1);

        private final DoubleSupplier setpointRadPerSec;
    }


    @Getter
    @Setter
    private IntakeRollerGoal intakeRollerGoal = IntakeRollerGoal.IDLE;

    private static IntakeRoller instance;

    public static IntakeRoller get() {
        if (instance == null)
            synchronized (IntakeRoller.class) {
                instance = new IntakeRoller();
            }

        return instance;
    }

    private IntakeRoller() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);

        Logger.processInputs("Inputs/IntakeRoller", inputs);

        robotMechanism.intakeRoller.topRollersLigament.setAngle
                (Units.radiansToDegrees(inputs.positionRad));
        intakeRollerPositionGainsTunable.ifChanged(io::setPositionPIDF);
        intakeRollerVelocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("IntakeRoller/Goal", intakeRollerGoal);
        if (intakeRollerGoal.setpointRadPerSec != null) {
            var rollersSetpointRadPerSec = intakeRollerGoal.setpointRadPerSec.getAsDouble();
            io.setVelocity(rollersSetpointRadPerSec);
            Logger.recordOutput("IntakeRoller/ClosedLoop", true);
            Logger.recordOutput("IntakeRoller/SetpointRadPerSec", rollersSetpointRadPerSec);
        } else {
            Logger.recordOutput("IntakeRoller/ClosedLoop", false);
        }

        if (defaultCommand != null && !defaultCommand.isScheduled()) {
            defaultCommand.schedule();

        }
    }
//
//    @AutoLogOutput(key = "IntakePivot/CurrentGoal")
//    public IntakeRollerGoal getCurrentGoal() {
//        return intakeRollerGoal;
//    }

    //    public void setRunning(boolean runIntake) {
//        io.setRunning(runIntake);
//    }
//
    public void spawnCoral() {
        io.spawnCoral();
    }

    public boolean hasCoral() {
        return inputs.isCoralIn;
    }


    public void setDefaultCommand(Command command) {
        this.defaultCommand = command;
        if (command != null) {
            command.schedule();
        }
    }

    public Command setGoals(IntakeRollerGoal intakeRollerGoal) {
        return runOnce(() -> {
            this.intakeRollerGoal = intakeRollerGoal;
        });
    }


}

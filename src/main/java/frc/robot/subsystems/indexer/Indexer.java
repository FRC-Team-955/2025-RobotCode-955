package frc.robot.subsystems.indexer;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.funnel.FunnelTuning.*;
import static frc.robot.subsystems.indexer.IndexerConstants.createIO;
import static frc.robot.subsystems.indexer.IndexerConstants.tolerances;

public class Indexer implements Periodic {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        INTAKE_ALTERNATE(() -> Timer.getTimestamp() % 1.0 < 0.92 ? intakeGoalSetpoint.get() : -intakeGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(() -> Timer.getTimestamp() % 1.0 < 0.86 ? ejectGoalSetpoint.get() : -ejectGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Setter
    @Getter
    private Goal goal = Goal.IDLE;

    private final Alert motorDisconnectedAlert = new Alert("Indexer motor is disconnected.", Alert.AlertType.kError);

    private static Indexer instance;

    public static Indexer get() {
        if (instance == null)
            synchronized (Indexer.class) {
                instance = new Indexer();
            }

        return instance;
    }

    private Indexer() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Indexer", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        robotMechanism.funnel.beltLigament.setAngle(Units.radiansToDegrees(-inputs.positionRad));

        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        Logger.recordOutput("Indexer/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("Indexer/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Indexer/RequestValue", value);
            io.setRequest(goal.type, value);
        }
    }

    @AutoLogOutput(key = "Indexer/AtGoal")
    public boolean atGoal() {
        double value = goal.value.getAsDouble();
        return switch (goal.type) {
            case PositionRad -> Math.abs(inputs.positionRad - value) <= tolerances.positionToleranceRad();

            case VelocityRadPerSec ->
                    Math.abs(inputs.velocityRadPerSec - value) <= tolerances.velocityToleranceRadPerSec();

            case VoltageVolts -> Math.abs(inputs.appliedVolts - value) <= tolerances.voltageToleranceVolts();
        };
    }

    public Command waitUntilAtGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}

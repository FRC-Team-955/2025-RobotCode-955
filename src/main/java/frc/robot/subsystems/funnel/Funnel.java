package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.motor.*;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.funnel.FunnelTuning.*;

public class Funnel implements Periodic {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private final MotorIO io = FunnelConstants.createIO();
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

    @Getter
    @Setter
    private Goal goal = Goal.IDLE;

    private final RequestHelper requestHelper = new RequestHelper(inputs, RequestTolerances.defaults());

    private final Alert beltDisconnectedAlert = new Alert("Funnel belt motor is disconnected.", Alert.AlertType.kError);

    private static Funnel instance;

    public static Funnel get() {
        if (instance == null)
            synchronized (Funnel.class) {
                instance = new Funnel();
            }

        return instance;
    }

    private Funnel() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Funnel", inputs);

        beltDisconnectedAlert.set(!inputs.connected);

        Logger.recordOutput("Funnel/AtLastGoal", requestHelper.processAtLastGoal());

        robotMechanism.funnel.beltLigament.setAngle(Units.radiansToDegrees(-inputs.positionRad));

        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        Logger.recordOutput("Funnel/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(MotorIO.RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("Funnel/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("Funnel/RequestValue", value);
            requestHelper.convertAndApplyRequest(
                    goal.hashCode(),
                    goal.type,
                    value,
                    (newType, newValue) -> {
                        Logger.recordOutput("Funnel/RequestTypeConverted", newType);
                        Logger.recordOutput("Funnel/RequestValueConverted", newValue);
                        io.setRequest(newType, newValue);
                    }
            );
        }
    }

    public Command waitUntilAtLastGoal() {
        return Commands.waitUntil(() -> requestHelper.isAtLastGoal(goal.hashCode()));
    }
}

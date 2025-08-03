package frc.lib.motor;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public abstract class MotorSubsystem<Goal extends Enum<Goal> & MotorSubsystem.GoalInterface> implements Periodic {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    protected final MotorIO io;
    protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    public interface GoalInterface {
        RequestType getType();

        /** Should be constant for every loop cycle */
        DoubleSupplier getValue();
    }

    @Setter
    @Getter
    protected Goal goal;

    protected final String name;
    protected final RequestTolerances tolerances;

    protected final Alert motorDisconnectedAlert;

    protected MotorSubsystem(
            String name,
            RequestTolerances tolerances,
            MotorIO io,
            Goal initialGoal
    ) {
        this.name = name;
        this.tolerances = tolerances;
        this.io = io;
        this.goal = initialGoal;
        this.motorDisconnectedAlert = new Alert(name + " motor is disconnected.", Alert.AlertType.kError);
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/" + name, inputs);

        motorDisconnectedAlert.set(!inputs.connected);
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        Logger.recordOutput(name + "/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput(name + "/RequestType", goal.getType());
            double value = goal.getValue().getAsDouble();
            Logger.recordOutput(name + "/RequestValue", value);
            io.setRequest(goal.getType(), value);
        }
    }

    @AutoLogOutput
    public boolean atGoal() {
        double value = goal.getValue().getAsDouble();
        return switch (goal.getType()) {
            case PositionRad -> Math.abs(inputs.positionRad - value) <= tolerances.positionToleranceRad();

            case VelocityRadPerSec ->
                    Math.abs(inputs.velocityRadPerSec - value) <= tolerances.velocityToleranceRadPerSec();

            case VoltageVolts -> Math.abs(inputs.appliedVolts - value) <= tolerances.voltageToleranceVolts();
        };
    }

    public Command waitUntilAtLastGoal() {
        return Commands.waitUntil(this::atGoal);
    }
}

package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
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
import frc.robot.subsystems.elevator.Elevator;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;
import static frc.robot.subsystems.endeffector.EndEffectorTuning.*;

public class EndEffector implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Elevator elevator = Elevator.get();

    private final MotorIO io = EndEffectorConstants.createRollersIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        FUNNEL_INTAKE(funnelIntakeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL(scoreCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL_L1(scoreCoralL1GoalSetpoint::get, RequestType.VelocityRadPerSec),
        DESCORE_ALGAE(descoreAlgaeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(() -> Timer.getTimestamp() % 1.0 < 0.86 ? ejectGoalSetpoint.get() : -ejectGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        ZERO_CORAL(zeroCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        HOME_INITIAL(() -> rollersRadiansForMeters(homeInitialMeters.get()), RequestType.RelativePositionRad),
        HOME_FINAL(() -> rollersRadiansForMeters(homeFinalMeters.get()), RequestType.RelativePositionRad),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Getter
    @Setter
    private Goal goal = Goal.IDLE;

    private final RequestHelper requestHelper = new RequestHelper(inputs, RequestTolerances.position(rollersPositionToleranceRad));

    private final Alert rollersDisconnectedAlert = new Alert("End effector rollers motor is disconnected.", Alert.AlertType.kError);

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
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/EndEffector", inputs);

        rollersDisconnectedAlert.set(!inputs.connected);

        Logger.recordOutput("EndEffector/AtLastGoal", requestHelper.processAtLastGoal());

        // Update mechanism
        robotMechanism.endEffector.ligament.setAngle(180 - Units.radiansToDegrees(getAngleRad()));
        // top rollers are reversed relative to motor
        robotMechanism.endEffector.topRollersLigament.setAngle(Units.radiansToDegrees(-inputs.positionRad));

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        positionGainsTunable.ifChanged(io::setPositionPIDF);
        velocityGainsTunable.ifChanged(io::setVelocityPIDF);
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("EndEffector/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(MotorIO.RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("EndEffector/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("EndEffector/RequestValue", value);
            requestHelper.convertAndApplyRequest(
                    goal.hashCode(),
                    goal.type,
                    value,
                    (newType, newValue) -> {
                        Logger.recordOutput("EndEffector/RequestTypeConverted", newType);
                        Logger.recordOutput("EndEffector/RequestValueConverted", newValue);
                        io.setRequest(newType, newValue);
                    }
            );
        }
    }

    public Command waitUntilAtLastGoal() {
        return Commands.waitUntil(() -> requestHelper.isAtLastGoal(goal.hashCode()));
    }

    @AutoLogOutput(key = "EndEffector/DescoreAlgaeAmperageTriggered")
    private boolean descoreAlgaeAmperageTriggered() {
        return Math.abs(inputs.currentAmps) > descoreAlgaeTriggerAmps;
    }

    public Command waitUntilDescoreAlgaeAmperageTriggered() {
        return Commands.waitUntil(this::descoreAlgaeAmperageTriggered);
    }

    @AutoLogOutput(key = "EndEffector/AngleRad")
    public double getAngleRad() {
        return MathUtil.interpolate(
                angleWhenRetractedRad,
                angleWhenExtendedRad,
                (elevator.getPositionMeters() - extendStartMeters) / extendDistanceMeters
        );
    }
}
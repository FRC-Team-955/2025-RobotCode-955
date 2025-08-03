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

import java.util.function.DoubleUnaryOperator;

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
        IDLE(t -> 0, RequestType.VoltageVolts),
        FUNNEL_INTAKE(t -> funnelIntakeGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        SCORE_CORAL(t -> scoreCoralGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        SCORE_CORAL_L1(t -> scoreCoralL1GoalSetpoint.get(), RequestType.VelocityRadPerSec),
        DESCORE_ALGAE(t -> descoreAlgaeGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(t -> t % 1.0 < 0.86 ? ejectGoalSetpoint.get() : -ejectGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        ZERO_CORAL(t -> zeroCoralGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        HOME_INITIAL(t -> rollersRadiansForMeters(homeInitialMeters.get()), RequestType.RelativePositionRad),
        HOME_FINAL(t -> rollersRadiansForMeters(homeFinalMeters.get()), RequestType.RelativePositionRad),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleUnaryOperator value;
        private final RequestType type;
    }

    @Getter
    @Setter
    private Goal goal = Goal.IDLE;

    private Goal lastGoal = goal;
    private final Timer goalTimer = new Timer();

    private final RequestHelper requestHelper = new RequestHelper(inputs, RequestTolerances.position(rollersPositionToleranceRad));
    private Double lastGoalValue = null;
    private boolean atLastGoal = false;

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

        if (lastGoalValue != null) {
            atLastGoal = requestHelper.atRequest(lastGoal.type, lastGoalValue);
            Logger.recordOutput("EndEffector/AtLastGoal", atLastGoal);
        }

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
            if (goal != lastGoal) {
                lastGoal = goal;
                goalTimer.restart();
            }

            Logger.recordOutput("EndEffector/RequestType", goal.type);
            lastGoalValue = goal.value.applyAsDouble(goalTimer.get());
            Logger.recordOutput("EndEffector/RequestValue", lastGoalValue);
            requestHelper.convertAndApplyRequest(
                    goal.hashCode(),
                    goal.type,
                    lastGoalValue,
                    (newType, newValue) -> {
                        Logger.recordOutput("EndEffector/RequestTypeConverted", newType);
                        Logger.recordOutput("EndEffector/RequestValueConverted", newValue);
                        io.setRequest(newType, newValue);
                    }
            );
        }
    }

    public Command waitUntilAtLastGoal() {
        // We don't want a false positive by looking at the previous goal
        return Commands.waitUntil(() -> goal == lastGoal && atLastGoal);
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
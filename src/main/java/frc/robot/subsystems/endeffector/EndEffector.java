package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
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
        HANDOFF(() -> 0, RequestType.VoltageVolts),
        FUNNEL_INTAKE(funnelIntakeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        FUNNEL_INTAKE_MANUAL(funnelIntakeManualGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL(scoreCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL_L1(scoreCoralL1GoalSetpoint::get, RequestType.VelocityRadPerSec),
        DESCORE_ALGAE(descoreAlgaeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(() -> {
            throw new RuntimeException("TODO alternate");
//            return ejectGoalSetpoint.get();
        }, RequestType.VelocityRadPerSec),
        ZERO_CORAL(zeroCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        HOME_INITIAL(() -> {
            throw new RuntimeException("TODO have a relative positive request type");
        }, RequestType.PositionRad),
        HOME_FINAL(() -> {
            throw new RuntimeException("TODO have a relative positive request type");
        }, RequestType.PositionRad),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Getter
    @Setter
    private Goal goal = Goal.IDLE;

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
        Logger.processInputs("Inputs/EndEffector/Rollers", inputs);

        rollersDisconnectedAlert.set(!inputs.connected);

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
            io.setRequest(goal.hashCode(), RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("EndEffector/RequestType", goal.type);
            double value = goal.value.getAsDouble();
            Logger.recordOutput("EndEffector/RequestValue", value);
            io.setRequest(goal.hashCode(), goal.type, value);
        }
    }

    @AutoLogOutput(key = "EndEffector/DescoreAlgaeAmperageTriggered")
    private boolean descoreAlgaeAmperageTriggered() {
        return Math.abs(inputs.currentAmps) > descoreAlgaeTriggerAmps;
    }

    public Command waitUntilDescoreAlgaeAmperageTriggered() {
        return Commands.waitUntil(this::descoreAlgaeAmperageTriggered);
    }

    /** Goes positionDeltaMeters forward (or backwards) from current position */
    public Command moveByAndWaitUntilDone(DoubleSupplier positionDeltaMeters) {
        throw new RuntimeException("TODO");
//        return startEndWaitUntil(
//                () -> {
//                    this.goal = Goal.GO_TO_POSITION;
//                    positionSetpointRad = inputs.positionRad + rollersRadiansForMeters(positionDeltaMeters.getAsDouble());
//                },
//                () -> {
//                    this.goal = Goal.IDLE;
//                    positionSetpointRad = null;
//                },
//                () -> Math.abs(inputs.positionRad - positionSetpointRad) <= rollersPositionToleranceRad
//        );
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
package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.motor.MotorSubsystem;
import frc.lib.motor.RequestTolerances;
import frc.lib.motor.RequestType;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.elevator.Elevator;
import lombok.Getter;
import lombok.NonNull;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;
import static frc.robot.subsystems.endeffector.EndEffectorTuning.*;

public class EndEffector extends MotorSubsystem<EndEffector.Goal> {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Elevator elevator = Elevator.get();

    @RequiredArgsConstructor
    @Getter
    public enum Goal implements GoalInterface {
        IDLE(() -> 0, RequestType.VoltageVolts),
        FUNNEL_INTAKE(funnelIntakeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL(scoreCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        SCORE_CORAL_L1(scoreCoralL1GoalSetpoint::get, RequestType.VelocityRadPerSec),
        DESCORE_ALGAE(descoreAlgaeGoalSetpoint::get, RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(() -> Timer.getTimestamp() % 1.0 < 0.86 ? ejectGoalSetpoint.get() : -ejectGoalSetpoint.get(), RequestType.VelocityRadPerSec),
        ZERO_CORAL(zeroCoralGoalSetpoint::get, RequestType.VelocityRadPerSec),
        HOME_INITIAL(() -> relativePositionOriginRad + rollersRadiansForMeters(homeInitialMeters.get()), RequestType.PositionRad),
        HOME_FINAL(() -> relativePositionOriginRad + rollersRadiansForMeters(homeFinalMeters.get()), RequestType.PositionRad),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    private static EndEffector instance;

    public static EndEffector get() {
        if (instance == null)
            synchronized (EndEffector.class) {
                instance = new EndEffector();
            }

        return instance;
    }

    private EndEffector() {
        super(
                "EndEffector",
                RequestTolerances.position(rollersPositionToleranceRad),
                createRollersIO(),
                Goal.IDLE
        );
    }

    @Override
    public void periodicBeforeCommands() {
        super.periodicBeforeCommands();

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

    @AutoLogOutput(key = "EndEffector/RelativePositionOriginRad")
    private static double relativePositionOriginRad = 0.0;

    @Override
    public void setGoal(@NonNull Goal goal) {
        super.setGoal(goal);
        if (goal == Goal.HOME_INITIAL || goal == Goal.HOME_FINAL) {
            relativePositionOriginRad = inputs.positionRad;
        }
    }
}
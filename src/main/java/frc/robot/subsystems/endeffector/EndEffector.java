package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.characterization.FeedforwardCharacterization;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.createRollersIO;
import static frc.robot.subsystems.endeffector.EndEffectorTuning.*;

public class EndEffector extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Elevator elevator = Elevator.get();

    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        HANDOFF(() -> 0),
        FUNNEL_INTAKE(funnelIntakeGoalSetpoint::get),
        ORIENT_CORAL(orientCoralGoalSetpoint::get),
        SCORE_CORAL(scoreCoralGoalSetpoint::get),
        DESCORE_ALGAE(descoreAlgaeGoalSetpoint::get);

        private final DoubleSupplier setpointRadPerSec;
    }

    @Getter
    private RollersGoal rollersGoal = RollersGoal.IDLE;

    private final RollersIO rollersIO = createRollersIO();
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

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
        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/EndEffector/Rollers", rollersInputs);

        rollersDisconnectedAlert.set(!rollersInputs.connected);

        robotMechanism.endEffector.ligament.setAngle(getAngleDegrees());
        robotMechanism.endEffector.ligament.setAngle(getAngleDegrees());
        // top rollers are reversed relative to motor
        robotMechanism.endEffector.topRollersLigament.setAngle(Units.radiansToDegrees(-rollersInputs.positionRad));
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged(hashCode())) {
            rollersIO.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        positionGainsTunable.ifChanged(hashCode(), rollersIO::setPositionPIDF);
        velocityGainsTunable.ifChanged(hashCode(), rollersIO::setVelocityPIDF);

        ////////////// ROLLERS //////////////
        Logger.recordOutput("EndEffector/Rollers/Goal", rollersGoal);
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("EndEffector/Rollers/ClosedLoop", false);
            rollersIO.setOpenLoop(0);
        } else if (rollersGoal.setpointRadPerSec != null) {
            var rollersSetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
            rollersIO.setVelocity(rollersSetpointRadPerSec);
            Logger.recordOutput("EndEffector/Rollers/ClosedLoop", true);
            Logger.recordOutput("EndEffector/Rollers/SetpointRadPerSec", rollersSetpointRadPerSec);
        } else {
            Logger.recordOutput("EndEffector/Rollers/ClosedLoop", false);
        }
    }

    public Command setGoal(RollersGoal rollersGoal) {
        return runOnce(() -> this.rollersGoal = rollersGoal);
    }

    public double getAngleDegrees() {
        return MathUtil.clamp(
                // After 5 inches, interpolate to 40 degrees finishing at 7.25 inches
                90 + (40 / Units.inchesToMeters(2.25) * (elevator.getPositionMeters() - Units.inchesToMeters(5))),
                90, 130
        );
    }

    public Command rollersFeedforwardCharacterization() {
        return setGoal(RollersGoal.CHARACTERIZATION)
                .andThen(new FeedforwardCharacterization(
                        rollersIO::setOpenLoop,
                        () -> new double[]{rollersInputs.velocityRadPerSec},
                        1,
                        this
                ));
    }
}
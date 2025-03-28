package frc.robot.subsystems.endeffector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.characterization.FeedforwardCharacterization;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.endeffector.EndEffectorConstants.*;
import static frc.robot.subsystems.endeffector.EndEffectorTuning.*;

public class EndEffector extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final Elevator elevator = Elevator.get();

    private final RollersIO rollersIO = createRollersIO();
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum RollersGoal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        HANDOFF(() -> 0),
        FUNNEL_INTAKE(funnelIntakeGoalSetpoint::get),
        SCORE_CORAL(scoreCoralGoalSetpoint::get),
        SCORE_CORAL_L1(scoreCoralL1GoalSetpoint::get),
        DESCORE_ALGAE(descoreAlgaeGoalSetpoint::get),
        EJECT(ejectGoalSetpoint::get),
        ZERO_CORAL(zeroCoralGoalSetpoint::get),
        GO_TO_POSITION(null); // Handled specially in periodic and with rollersPositionSetpointRad

        private final DoubleSupplier setpointRadPerSec;
    }

    @Getter
    private RollersGoal rollersGoal = RollersGoal.IDLE;
    private Double rollersPositionSetpointRad = null;

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
        super(10);
    }

    @Override
    public void periodicBeforeCommands() {
        rollersIO.updateInputs(rollersInputs);
        Logger.processInputs("Inputs/EndEffector/Rollers", rollersInputs);

        rollersDisconnectedAlert.set(!rollersInputs.connected);

        robotMechanism.endEffector.ligament.setAngle(180 - Units.radiansToDegrees(getAngleRad()));
        // top rollers are reversed relative to motor
        robotMechanism.endEffector.topRollersLigament.setAngle(Units.radiansToDegrees(-rollersInputs.positionRad));
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged()) {
            rollersIO.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        positionGainsTunable.ifChanged(rollersIO::setPositionPIDF);
        velocityGainsTunable.ifChanged(rollersIO::setVelocityPIDF);

        ////////////// ROLLERS //////////////
        Logger.recordOutput("EndEffector/Rollers/Goal", rollersGoal);
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", false);
            rollersIO.setOpenLoop(0);
        } else if (rollersGoal.setpointRadPerSec != null) {
            // Velocity control
            var rollersVelocitySetpointRadPerSec = rollersGoal.setpointRadPerSec.getAsDouble();
            rollersIO.setClosedLoopVelocity(rollersVelocitySetpointRadPerSec);
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", true);
            Logger.recordOutput("EndEffector/Rollers/Velocity/SetpointRadPerSec", rollersVelocitySetpointRadPerSec);
        } else if (rollersGoal == RollersGoal.GO_TO_POSITION && rollersPositionSetpointRad != null) {
            // Position control
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", true);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Position/SetpointRad", rollersPositionSetpointRad);
            rollersIO.setClosedLoopPosition(rollersPositionSetpointRad);
        } else {
            Logger.recordOutput("EndEffector/Rollers/Position/ClosedLoop", false);
            Logger.recordOutput("EndEffector/Rollers/Velocity/ClosedLoop", false);
        }
    }

    @AutoLogOutput(key = "EndEffector/DescoreAlgaeAmperageTriggered")
    private boolean descoreAlgaeAmperageTriggered() {
        return Math.abs(rollersInputs.currentAmps) > descoreAlgaeTriggerAmps;
    }

    public Command waitUntilDescoreAlgaeAmperageTriggered() {
        return Commands.waitUntil(this::descoreAlgaeAmperageTriggered);
    }

    public Command setGoal(RollersGoal rollersGoal) {
        return runOnce(() -> this.rollersGoal = rollersGoal);
    }

    /** Goes positionDeltaMeters forward (or backwards) from current position */
    public Command moveByAndWaitUntilDone(DoubleSupplier positionDeltaMeters) {
        return startEndWaitUntil(
                () -> {
                    this.rollersGoal = RollersGoal.GO_TO_POSITION;
                    rollersPositionSetpointRad = rollersInputs.positionRad + rollersRadiansForMeters(positionDeltaMeters.getAsDouble());
                },
                () -> {
                    this.rollersGoal = RollersGoal.IDLE;
                    rollersPositionSetpointRad = null;
                },
                () -> Math.abs(rollersInputs.positionRad - rollersPositionSetpointRad) <= rollersPositionToleranceRad
        );
    }

    @AutoLogOutput(key = "EndEffector/AngleRad")
    public double getAngleRad() {
        return MathUtil.interpolate(
                angleWhenRetractedRad,
                angleWhenExtendedRad,
                (elevator.getPositionMeters() - extendStartMeters) / extendDistanceMeters
        );
    }

    public Command rollersFeedforwardCharacterization() {
        return CommandsExt.eagerSequence(
                setGoal(RollersGoal.CHARACTERIZATION),
                new FeedforwardCharacterization(
                        rollersIO::setOpenLoop,
                        () -> new double[]{rollersInputs.velocityRadPerSec},
                        1,
                        this
                )
        );
    }
}
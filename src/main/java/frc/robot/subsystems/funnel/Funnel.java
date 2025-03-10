package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOInputsAutoLogged;
import frc.robot.util.characterization.FeedforwardCharacterization;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.funnel.FunnelConstants.createBeltIO;
import static frc.robot.subsystems.funnel.FunnelTuning.*;

public class Funnel extends SubsystemBaseExt {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private final RollersIO beltIO = createBeltIO();
    private final RollersIOInputsAutoLogged beltInputs = new RollersIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        CHARACTERIZATION(null),
        IDLE(() -> 0),
        INTAKE_FORWARDS(intakeGoalSetpoint::get),
        INTAKE_BACKWARDS(() -> -intakeGoalSetpoint.get()),
        EJECT_FORWARDS(ejectGoalSetpoint::get),
        EJECT_BACKWARDS(() -> -ejectGoalSetpoint.get());

        private final DoubleSupplier setpointRadPerSec;
    }

    @Getter
    private Goal goal = Goal.IDLE;

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
        beltIO.updateInputs(beltInputs);
        Logger.processInputs("Inputs/Funnel/Belt", beltInputs);

        beltDisconnectedAlert.set(!beltInputs.connected);

        robotMechanism.funnel.beltLigament.setAngle(Units.radiansToDegrees(-beltInputs.positionRad));
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged(hashCode())) {
            beltIO.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        velocityGainsTunable.ifChanged(hashCode(), beltIO::setVelocityPIDF);

        Logger.recordOutput("Funnel/Goal", goal);
        ////////////// BELT //////////////
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Funnel/Belt/ClosedLoop", false);
            beltIO.setOpenLoop(0);
        } else if (goal.setpointRadPerSec != null) {
            // Velocity control
            var beltVelocitySetpointRadPerSec = goal.setpointRadPerSec.getAsDouble();
            beltIO.setVelocity(beltVelocitySetpointRadPerSec);
            Logger.recordOutput("Funnel/Belt/ClosedLoop", true);
            Logger.recordOutput("Funnel/Belt/SetpointRadPerSec", beltVelocitySetpointRadPerSec);
        } else {
            Logger.recordOutput("Funnel/Belt/ClosedLoop", false);
        }
    }

    public Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

    public void setGoalInstantaneous(Goal goal) {
        this.goal = goal;
    }

    public Command beltFeedforwardCharacterization() {
        return setGoal(Goal.CHARACTERIZATION)
                .andThen(new FeedforwardCharacterization(
                        beltIO::setOpenLoop,
                        () -> new double[]{beltInputs.velocityRadPerSec},
                        1,
                        this
                ));
    }
}

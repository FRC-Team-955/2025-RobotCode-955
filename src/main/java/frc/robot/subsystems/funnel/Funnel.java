package frc.robot.subsystems.funnel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestConverter;
import frc.lib.motor.RequestType;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.funnel.FunnelTuning.velocityGainsTunable;

public class Funnel implements Periodic {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();
    private final RobotMechanism robotMechanism = RobotMechanism.get();

    private final MotorIO io = FunnelConstants.createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        IDLE(() -> 0, RequestType.VoltageVolts),
        INTAKE_ALTERNATE(() -> {
            throw new RuntimeException("TODO alternate, manual intaking");
//            return intakeGoalSetpoint.get();
        }, RequestType.VelocityRadPerSec),
        EJECT_ALTERNATE(() -> {
            throw new RuntimeException("TODO alternate");
//            return ejectGoalSetpoint.get();
        }, RequestType.VelocityRadPerSec),
        ;

        /** Should be constant for every loop cycle */
        private final DoubleSupplier value;
        private final RequestType type;
    }

    @Getter
    @Setter
    private Goal goal = Goal.IDLE;

    private final RequestConverter requestConverter = new RequestConverter(inputs);

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
        Logger.processInputs("Inputs/Funnel/Belt", inputs);

        beltDisconnectedAlert.set(!inputs.connected);

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
            requestConverter.convertAndApplyRequest(
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
}

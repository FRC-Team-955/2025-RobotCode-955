package frc.robot.subsystems.intakepivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.motor.MotorIO;
import frc.lib.motor.MotorIOInputsAutoLogged;
import frc.lib.motor.RequestType;
import frc.lib.subsystem.Periodic;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.intakepivot.IntakePivotConstants.createIO;
import static frc.robot.subsystems.intakepivot.IntakePivotTuning.gainsTunable;

public class IntakePivot implements Periodic {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final MotorIO io = createIO();
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        STOW(1),
        DEPLOY(0),
        ;

        private final double setpointRad;
    }

    @Getter
    private Goal goal = Goal.STOW;

    private final Alert motorDisconnectedAlert = new Alert("Intake pivot motor is disconnected", Alert.AlertType.kError);

    private static IntakePivot instance;

    public static IntakePivot get() {
        if (instance == null) {
            synchronized (IntakePivot.class) {
                instance = new IntakePivot();
            }
        }

        return instance;
    }

    private IntakePivot() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/IntakePivot", inputs);

        motorDisconnectedAlert.set(!inputs.connected);

        // Update mechanism

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        gainsTunable.ifChanged(io::setPositionPIDF);
    }

    @Override
    public void periodicAfterCommands() {

        if (Timer.getTimestamp() % 2 < 1) {
            goal = IntakePivot.Goal.STOW;
        } else {
            goal = IntakePivot.Goal.DEPLOY;
        }

        Logger.recordOutput("IntakePivot/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            Logger.recordOutput("IntakePivot/SetpointRad", goal.setpointRad);
            io.setRequest(RequestType.PositionRad, goal.setpointRad);
        }
    }
}

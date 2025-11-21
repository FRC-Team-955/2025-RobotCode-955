package frc.robot.subsystems.intakepivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    final Timer t = new Timer();
    double start = 0;
    final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            1,
            3
    ));
    TrapezoidProfile.State last = new TrapezoidProfile.State();
    TrapezoidProfile.State last2 = new TrapezoidProfile.State();

    @Override
    public void periodicAfterCommands() {
        if (Timer.getTimestamp() % 4 < 2) {
            if (goal != Goal.STOW) {
                t.restart();
                start = inputs.positionRad;
                last = profile.calculate(0.15, last, new TrapezoidProfile.State(Goal.STOW.setpointRad, 0));
            }
            goal = IntakePivot.Goal.STOW;
        } else {
            if (goal != Goal.DEPLOY) {
                t.restart();
                start = inputs.positionRad;
                last = profile.calculate(0.15, last, new TrapezoidProfile.State(Goal.DEPLOY.setpointRad, 0));
            }
            goal = IntakePivot.Goal.DEPLOY;
        }

        Logger.recordOutput("IntakePivot/Goal", goal);
        if (DriverStation.isDisabled()) {
            io.setRequest(RequestType.VoltageVolts, 0);
        } else {
            last2 = profile.calculate(0.02, last2, new TrapezoidProfile.State(goal.setpointRad, 0));
            last = profile.calculate(0.02, last, new TrapezoidProfile.State(goal.setpointRad, 0));
            Logger.recordOutput("IntakePivot/OrigSetpointRad", last2.position);

            double setpoint = last.position + Math.copySign(0.1, goal.setpointRad - inputs.positionRad);
            if (
                    (goal.setpointRad > inputs.positionRad && setpoint > goal.setpointRad) ||
                            (goal.setpointRad < inputs.positionRad && setpoint < goal.setpointRad)
            ) {
                setpoint = goal.setpointRad;
            }
            setpoint = last.position;

//            double setpoint = start + t.get() * t.get() * Math.copySign(0.5, goal.setpointRad - inputs.positionRad);
//            if (
//                    Math.abs(goal.setpointRad - inputs.positionRad) < 0.2 ||
//                            (goal.setpointRad > inputs.positionRad && setpoint > goal.setpointRad) ||
//                            (goal.setpointRad < inputs.positionRad && setpoint < goal.setpointRad)
//            ) {
//                setpoint = goal.setpointRad;
//            }

//            double setpoint = goal.setpointRad;
//            if (Math.abs(goal.setpointRad - inputs.positionRad) > 0.2) {
//                setpoint = inputs.positionRad + Math.copySign(0.2, goal.setpointRad - inputs.positionRad);
//            }
            Logger.recordOutput("IntakePivot/SetpointRad", setpoint);
            io.setRequest(RequestType.PositionRad, setpoint);
        }
    }
}

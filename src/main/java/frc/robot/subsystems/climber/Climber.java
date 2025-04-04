package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.RobotMechanism;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.climber.ClimberConstants.createIO;
import static frc.robot.subsystems.climber.ClimberConstants.gains;
import static frc.robot.subsystems.climber.ClimberTuning.gainsTunable;
import static frc.robot.subsystems.climber.ClimberTuning.stowGoalSetpoint;

public class Climber extends SubsystemBaseExt {
    private final RobotMechanism robotMechanism = RobotMechanism.get();
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final ClimberIO io = createIO();
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    @RequiredArgsConstructor
    public enum Goal {
        STOW(stowGoalSetpoint::get, GoalType.Position),
        HOLD(() -> 0, GoalType.Voltage),
        CLIMB_TOWARDS_ROBOT(() -> 12, GoalType.Voltage),
        CLIMB_AWAY_FROM_ROBOT(() -> -12, GoalType.Voltage);

        public final DoubleSupplier value;
        public final GoalType type;
    }

    public enum GoalType {
        Voltage,
        Position,
    }

    @Getter
    private Goal goal = Goal.STOW;

    private final PIDController controller = gains.toPIDWrapRadians();

    private final Alert disconnectedMotorAlert = new Alert("Climber motor is disconnected.", Alert.AlertType.kError);
    private final Alert disconnectedAbsoluteEncoderAlert = new Alert("Climber absolute encoder is disconnected.", Alert.AlertType.kError);
    private final Alert temperatureAlert = new Alert("Climber motor temperature is high.", Alert.AlertType.kWarning);

    private static Climber instance;

    public static Climber get() {
        if (instance == null)
            synchronized (Climber.class) {
                instance = new Climber();
            }

        return instance;
    }

    private Climber() {
        super(10);
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);

        // Update alerts
        disconnectedMotorAlert.set(!inputs.connected);
        disconnectedAbsoluteEncoderAlert.set(!inputs.absoluteEncoderConnected);
        temperatureAlert.set(inputs.temperatureCelsius > 35);

        // Update mechanism
        robotMechanism.climber.ligament.setAngle(Units.radiansToDegrees(inputs.absolutePositionRad));

        // Apply network inputs
        if (operatorDashboard.coastOverride.hasChanged()) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }

        gainsTunable.ifChanged(gains -> gains.applyPID(controller));
    }

    @Override
    public void periodicAfterCommands() {
        Logger.recordOutput("Climber/Goal", goal);
        if (DriverStation.isDisabled() || operatorDashboard.climberEStop.get()) {
            Logger.recordOutput("Climber/Running", false);
            io.setVoltage(0);
            controller.reset();
        } else {
            Logger.recordOutput("Climber/Running", true);
            double value = goal.value.getAsDouble();
            switch (goal.type) {
                case Voltage -> io.setVoltage(value);
                case Position -> io.setVoltage(
                        inputs.absoluteEncoderConnected
                                ? controller.calculate(inputs.absolutePositionRad, value)
                                : 0
                );
            }
        }
    }

    public Command setGoal(Supplier<Goal> goal) {
        return runOnce(() -> this.goal = goal.get());
    }
}
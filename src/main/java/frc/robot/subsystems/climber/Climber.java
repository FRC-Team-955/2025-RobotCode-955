package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OperatorDashboard;
import frc.robot.util.subsystem.SubsystemBaseExt;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.climber.ClimberConstants.createIO;


public class Climber extends SubsystemBaseExt {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final ClimberIO io = createIO();
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final Alert disconnectedAlert = new Alert("Climber motor motor is disconnected.", Alert.AlertType.kError);
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
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/Climber", inputs);

        disconnectedAlert.set(!inputs.connected);
        temperatureAlert.set(inputs.temperatureCelsius > 30);
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged(hashCode())) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }
    }

    public Command moveTowardsRobot() {
        return startEnd(
                () -> io.setOpenLoop(3),
                () -> io.setOpenLoop(0)
        );
    }

    public Command moveAwayFromRobot() {
        return startEnd(
                () -> io.setOpenLoop(-3),
                () -> io.setOpenLoop(0)
        );
    }
}
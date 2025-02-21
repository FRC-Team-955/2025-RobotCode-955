package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import frc.robot.OperatorDashboard;
import frc.robot.util.subsystem.SubsystemBaseExt;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.climber.ClimberConstants.createIO;


public class Climber extends SubsystemBaseExt {
    private final OperatorDashboard operatorDashboard = OperatorDashboard.get();

    private final ClimberIO io = createIO();
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final Alert temperatureAlert = new Alert("Climber motor temperature is high. Be careful.", Alert.AlertType.kWarning);

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

        temperatureAlert.set(inputs.temperatureCelsius > 30);
    }

    @Override
    public void periodicAfterCommands() {
        if (operatorDashboard.coastOverride.hasChanged(hashCode())) {
            io.setBrakeMode(!operatorDashboard.coastOverride.get());
        }
    }
}
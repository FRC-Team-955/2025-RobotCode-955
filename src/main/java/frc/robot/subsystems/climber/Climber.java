package frc.robot.subsystems.climber;

import frc.robot.util.subsystem.SubsystemBaseExt;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.climber.ClimberConstants.createIO;


public class Climber extends SubsystemBaseExt {
    private final ClimberIO io = createIO();
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

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
    }

    @Override
    public void periodicAfterCommands() {

    }
}
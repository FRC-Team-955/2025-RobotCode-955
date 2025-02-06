package frc.robot.subsystems.leds;

import frc.robot.util.SubsystemBaseExt;
import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBaseExt {
    private final LEDsIO io = LEDConstants.io;
    private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

    private static LEDs instance;

    public static LEDs get() {
        if (instance == null)
            synchronized (LEDs.class) {
                instance = new LEDs();
            }

        return instance;
    }

    private LEDs() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/LEDs", inputs);
    }

    @Override
    public void periodicAfterCommands() {

    }
}
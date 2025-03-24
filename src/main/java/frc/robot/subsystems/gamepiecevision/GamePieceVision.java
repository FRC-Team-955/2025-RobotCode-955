package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.commands.CommandsExt;
import frc.robot.util.subsystem.SubsystemBaseExt;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.createIO;

public class GamePieceVision extends SubsystemBaseExt {
    private final GamePieceVisionIO io = createIO();
    private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();

    private final Alert disconnectedAlert = new Alert("Game piece vision is disconnected.", Alert.AlertType.kError);

    private final Debouncer visibleDebouncer = new Debouncer(0.25);

    private static GamePieceVision instance;

    public static GamePieceVision get() {
        if (instance == null)
            synchronized (GamePieceVision.class) {
                instance = new GamePieceVision();
            }

        return instance;
    }

    private GamePieceVision() {
        super(2);
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/GamePieceVision", inputs);
        // Update disconnected alert
        disconnectedAlert.set(!inputs.connected);
    }

    @AutoLogOutput(key = "GamePieceVision/VisibleDebounced")
    public boolean visibleDebounced() {
        return visibleDebouncer.calculate(inputs.visible);
    }

    public Command waitForGamePiece() {
        return CommandsExt.startEndWaitUntil(
                () -> io.setLEDs(true),
                () -> io.setLEDs(false),
                this::visibleDebounced
        );
    }
}

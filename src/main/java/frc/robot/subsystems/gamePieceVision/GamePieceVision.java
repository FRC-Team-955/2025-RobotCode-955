package frc.robot.subsystems.gamePieceVision;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.subsystem.Periodic;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static frc.robot.subsystems.gamePieceVision.GamePieceVisionConstants.createIO;

public class GamePieceVision implements Periodic {

    public final Drive drive = Drive.get();

    private final GamePieceVisionIO io = createIO();
    private final GamePieceVisionIOInputsAutoLogged
            inputs = new GamePieceVisionIOInputsAutoLogged();

    private final Debouncer visibleDebouncer = new Debouncer(0.05);

    private static GamePieceVision instance;

    public static GamePieceVision get() {
        if (instance == null) {
            synchronized (GamePieceVision.class) {
                instance = new GamePieceVision();
            }
        }
        return instance;
    }

    private GamePieceVision() {
    }

    @Override
    public void periodicBeforeCommands() {


        io.updateInputs(inputs);
        Logger.processInputs("Inputs/GamePieceVision", inputs);
        boolean stableVisible = visibleDebouncer.calculate(inputs.visible);
        inputs.visible = stableVisible;

    }

    @Override
    public void periodicAfterCommands() {

    }

    //
    public Translation2d getCoralRobotRelative() {

        return inputs.coralRobotRelative;
    }

    public boolean getVisible() {
        return inputs.visible;
    }

    public Rotation2d getCoralYaw() {
        return inputs.coralYaw;
    }


    public Supplier<Pose2d> getCoralPos() {
        return () -> inputs.coralPos;
    }


}

package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.camera;
import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.createIO;

public class GamePieceVision implements Periodic {
    private final RobotState robotState = RobotState.get();

    private final GamePieceVisionIO io = createIO();
    private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();

    private final Alert disconnectedAlert = new Alert("Game piece vision is disconnected.", Alert.AlertType.kError);

    private static GamePieceVision instance;

    public static GamePieceVision get() {
        if (instance == null)
            synchronized (GamePieceVision.class) {
                instance = new GamePieceVision();
            }

        return instance;
    }

    private GamePieceVision() {
    }

    @Override
    public void periodicBeforeCommands() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/GamePieceVision", inputs);

        // Update disconnected alert
        disconnectedAlert.set(!inputs.connected);
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera pose for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput("GamePieceVision/CameraPose", robotPose.transformBy(camera.robotToCamera()));
    }
}

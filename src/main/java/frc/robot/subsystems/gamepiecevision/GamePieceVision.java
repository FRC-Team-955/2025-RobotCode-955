package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.subsystems.gamepiecevision.GamePieceVisionConstants.*;

public class GamePieceVision implements Periodic {
    private final RobotState robotState = RobotState.get();

    private final GamePieceVisionIO io = createIO();
    private final GamePieceVisionIOInputsAutoLogged inputs = new GamePieceVisionIOInputsAutoLogged();

    private final Alert disconnectedAlert = new Alert("Game piece vision is disconnected.", Alert.AlertType.kError);

    private final Map<Pose3d, Double> seenCoralToLastSeen = new HashMap<>();

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

        // Process observations
        seenCoralToLastSeen.clear();
        for (var observation : inputs.targetObservations) {
            // Calculate position
            double camToCoralZ = -camera.robotToCamera().getZ() + coralHeightMeters / 2.0;
            double camToCoralX = -camToCoralZ / observation.pitch().plus(Rotation2d.fromRadians(camera.robotToCamera().getRotation().getY())).getTan();
            Transform3d camToCoral = new Transform3d(
                    new Translation3d(
                            camToCoralX,
                            camToCoralX * observation.yaw().getTan(),
                            camToCoralZ
                    ),
                    new Rotation3d()
            );
            Transform3d robotToCoral = new Transform3d(camera.robotToCamera().getTranslation(), new Rotation3d()).plus(camToCoral);
            seenCoralToLastSeen.put(new Pose3d(robotState.getPose()).transformBy(robotToCoral), observation.timestamp());
        }

        // TODO: remove coral in same position
        // TODO: remove coral seen too long ago

        // Log seen coral
        Logger.recordOutput("GamePieceVision/SeenCoral", seenCoralToLastSeen.keySet().toArray(Pose3d[]::new));
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera pose for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput("GamePieceVision/CameraPose", robotPose.transformBy(camera.robotToCamera()));
    }
}

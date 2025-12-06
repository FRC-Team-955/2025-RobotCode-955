package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
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

        seenCoralToLastSeen.clear();
        List<Translation2d> targetPoints = new LinkedList<>();

        // Process observations
        var robotPose = new Pose3d(robotState.getPose());
        for (var observation : inputs.targetObservations) {
            targetPoints.add(new Translation2d(observation.yawRad(), observation.pitchRad()));

            // First, calculate position of target in camera space
            double camToTargetZ = -camera.robotToCamera().getZ() + coralHeightMeters / 2.0;
            // Account for pitch of camera
            double camToTargetX = camToTargetZ / Math.tan(observation.pitchRad() - cameraOrientation.pitchRad);
            double camToTargetY = camToTargetX * Math.tan(-observation.yawRad());

            // Next, translate x and y to robot coordinates
            Translation2d camToTargetXY = new Translation2d(camToTargetX, camToTargetY);
            Translation2d robotToTargetXY = camera.robotToCamera().getTranslation().toTranslation2d()
                    .plus(camToTargetXY)
                    // Account for yaw of camera
                    .rotateBy(Rotation2d.fromRadians(cameraOrientation.yawRad));
            double robotToTargetZ = camToTargetZ + camera.robotToCamera().getZ();

            Translation3d robotToTarget = new Translation3d(robotToTargetXY.getX(), robotToTargetXY.getY(), robotToTargetZ);

            seenCoralToLastSeen.put(
                    robotPose.transformBy(new Transform3d(robotToTarget, new Rotation3d())),
                    observation.timestamp()
            );
        }

        // TODO: remove coral in same position
        // TODO: remove coral seen too long ago

        // Log results
        Logger.recordOutput("GamePieceVision/TargetPoints", targetPoints.toArray(Translation2d[]::new));
        Logger.recordOutput("GamePieceVision/SeenCoral", seenCoralToLastSeen.keySet().toArray(Pose3d[]::new));
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera pose for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput("GamePieceVision/CameraPose", robotPose.transformBy(camera.robotToCamera()));
    }
}

package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import lombok.Getter;
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

    private final Map<Pose3d, Double> coralPoseToLastSeen = new HashMap<>();
    @Getter
    private List<Pose3d> freshCoral = List.of();
    @Getter
    private List<Pose3d> staleCoral = List.of();

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

        Map<Pose3d, Double> newlySeenCoral = new HashMap<>();
        List<Translation2d> targetPoints = new LinkedList<>();

        // Process observations
        var robotPose = new Pose3d(robotState.getPose());
        for (var observation : inputs.targetObservations) {
            Translation2d targetYawPitch = new Translation2d(observation.yawRad(), observation.pitchRad());
            targetPoints.add(targetYawPitch);

            // Account for roll of camera
            targetYawPitch = targetYawPitch.rotateBy(Rotation2d.fromRadians(-cameraOrientation.rollRad));
            double targetYaw = targetYawPitch.getX();
            double targetPitch = targetYawPitch.getY();

            // First, calculate position of target in camera space
            double camToTargetZ = -robotToCamera.getZ() + coralHeightMeters / 2.0;
            // Account for pitch of camera
            double camToTargetX = camToTargetZ / Math.tan(targetPitch - cameraOrientation.pitchRad);
            double camToTargetY = camToTargetX * Math.tan(-targetYaw);

            // Next, translate x and y to robot coordinates
            Translation2d camToTargetXY = new Translation2d(camToTargetX, camToTargetY);
            Translation2d robotToTargetXY = robotToCamera.getTranslation().toTranslation2d()
                    .plus(camToTargetXY)
                    // Account for yaw of camera
                    .rotateBy(Rotation2d.fromRadians(cameraOrientation.yawRad));
            double robotToTargetZ = camToTargetZ + robotToCamera.getZ();

            Translation3d robotToTarget = new Translation3d(robotToTargetXY.getX(), robotToTargetXY.getY(), robotToTargetZ);

            newlySeenCoral.put(
                    robotPose.transformBy(new Transform3d(robotToTarget, new Rotation3d())),
                    observation.timestamp()
            );
        }

        // Handle newly seen coral
        for (var pose : newlySeenCoral.keySet()) {
            // Remove old coral within distance to be counted as the same piece of coral
            coralPoseToLastSeen.keySet()
                    .removeIf(otherPose -> pose.getTranslation().getDistance(otherPose.getTranslation()) < minDistanceForSameCoralMeters);
        }
        coralPoseToLastSeen.putAll(newlySeenCoral);

        // Remove expired coral
        coralPoseToLastSeen.values().removeIf(lastSeen -> Timer.getTimestamp() - lastSeen > seenCoralExpireTimeSeconds);

        // Generate fresh/stale arrays
        freshCoral = new LinkedList<>();
        staleCoral = new LinkedList<>();
        for (var entry : coralPoseToLastSeen.entrySet()) {
            Pose3d pose = entry.getKey();
            double lastSeen = entry.getValue();

            if (Timer.getTimestamp() - lastSeen < seenCoralTimeForRecent) {
                freshCoral.add(pose);
            } else {
                staleCoral.add(pose);
            }
        }

        // Log results
        Logger.recordOutput("GamePieceVision/TargetPoints", targetPoints.toArray(Translation2d[]::new));
        Logger.recordOutput("GamePieceVision/FreshCoral", freshCoral.toArray(Pose3d[]::new));
        Logger.recordOutput("GamePieceVision/StaleCoral", staleCoral.toArray(Pose3d[]::new));
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera pose for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput("GamePieceVision/CameraPose", robotPose.transformBy(robotToCamera));
    }
}

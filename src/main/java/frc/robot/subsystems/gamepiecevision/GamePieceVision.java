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
        List<Pose3d> pointingAtCorals = new LinkedList<>();

        // Process observations
        var robotPose = new Pose3d(robotState.getPose());
        for (var observation : inputs.targetObservations) {
            // Rotate camera by pitch and yaw so that it points at coral
            Rotation3d pointingAtCoral = camera.robotToCamera().getRotation()
                    // rotation order is yaw-pitch (Tait-Bryan angles without roll)
                    .rotateBy(new Rotation3d(0, 0, observation.yawRad()))
                    .rotateBy(new Rotation3d(0, observation.pitchRad(), 0));

            // Log camera pointing at coral for debugging
            Transform3d robotToCameraPointingAtCoral = new Transform3d(camera.robotToCamera().getTranslation(), pointingAtCoral);
            pointingAtCorals.add(robotPose.transformBy(robotToCameraPointingAtCoral));
            for (int i = 0; i < 20; i++) {
                double j = i / 10.0;
                Transform3d a = robotToCameraPointingAtCoral.plus(new Transform3d(j, 0, 0, new Rotation3d()));
                pointingAtCorals.add(robotPose.transformBy(a));
            }

            // Get yaw and pitch from robot to camera translation to coral
            Quaternion q = pointingAtCoral.getQuaternion();
            // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_(in_3-2-1_sequence)_conversion
            double coralYaw = Math.atan2(2.0 * (q.getW() * q.getZ() + q.getX() * q.getY()), 1.0 - 2.0 * (q.getY() * q.getY() + q.getZ() * q.getZ()));
            double coralPitch = (-Math.PI / 2.0) + 2.0 * Math.atan2(Math.sqrt(1 + 2.0 * (q.getW() * q.getY() - q.getX() * q.getZ())), Math.sqrt(1 - 2.0 * (q.getW() * q.getY() - q.getX() * q.getZ())));

            // Calculate position
            double camToCoralZ = -camera.robotToCamera().getZ() + coralHeightMeters / 2.0;
            double camToCoralX = -camToCoralZ / Math.tan(coralPitch);
            Transform3d camToCoral = new Transform3d(
                    new Translation3d(
                            camToCoralX,
                            camToCoralX * Math.tan(coralYaw),
                            camToCoralZ
                    ),
                    new Rotation3d()
            );

            Transform3d robotToCoral = new Transform3d(
                    camera.robotToCamera().getTranslation(),
                    new Rotation3d()
            ).plus(camToCoral);
            seenCoralToLastSeen.put(new Pose3d(robotState.getPose()).transformBy(robotToCoral), observation.timestamp());
        }

        // TODO: remove coral in same position
        // TODO: remove coral seen too long ago

        // Log results
        Logger.recordOutput("GamePieceVision/PointingAtCorals", pointingAtCorals.toArray(Pose3d[]::new));
        Logger.recordOutput("GamePieceVision/SeenCoral", seenCoralToLastSeen.keySet().toArray(Pose3d[]::new));
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera pose for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput("GamePieceVision/CameraPose", robotPose.transformBy(camera.robotToCamera()));
    }
}

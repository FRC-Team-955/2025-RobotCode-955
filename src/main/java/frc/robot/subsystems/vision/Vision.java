// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.AprilTagIO.PoseObservationType;
import frc.robot.util.subsystem.SubsystemBaseExt;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.IntFunction;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class Vision extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();

    private final Map<AprilTagCamera, AprilTagCameraData> aprilTagCameras = Map.ofEntries(
            Arrays.stream(AprilTagCamera.values())
                    .map(cam -> Map.entry(cam, new AprilTagCameraData(
                            new AprilTagIOInputsAutoLogged(),
                            cam.createIO(),
                            new Alert("AprilTag camera " + cam.name() + " is disconnected.", AlertType.kError)
                    )))
                    .toArray((IntFunction<Map.Entry<AprilTagCamera, AprilTagCameraData>[]>) Map.Entry[]::new)
    );
    private final Map<GamepieceCamera, GamepieceCameraData> gamepieceCameras = Map.ofEntries(
            Arrays.stream(GamepieceCamera.values())
                    .map(cam -> Map.entry(cam, new GamepieceCameraData(
                            new GamepieceIOInputsAutoLogged(),
                            cam.createIO(),
                            new Alert("Gamepiece camera " + cam.name() + " is disconnected.", AlertType.kError)
                    )))
                    .toArray((IntFunction<Map.Entry<GamepieceCamera, GamepieceCameraData>[]>) Map.Entry[]::new)
    );

    @Getter
    private Optional<Translation2d> closestGamepiece = Optional.empty();

    private static Vision instance;

    public static Vision get() {
        if (instance == null)
            synchronized (Vision.class) {
                instance = new Vision();
            }

        return instance;
    }

    private Vision() {
    }

    @Override
    public void periodicBeforeCommands() {
        for (Map.Entry<AprilTagCamera, AprilTagCameraData> cam : aprilTagCameras.entrySet()) {
            AprilTagCamera metadata = cam.getKey();
            AprilTagCameraData data = cam.getValue();
            data.io.updateInputs(data.inputs);
            Logger.processInputs("Inputs/Vision/AprilTag/" + metadata.name(), data.inputs);
            // Update disconnected alert
            data.disconnectedAlert.set(!data.inputs.connected);
        }

        for (Map.Entry<GamepieceCamera, GamepieceCameraData> cam : gamepieceCameras.entrySet()) {
            GamepieceCamera metadata = cam.getKey();
            GamepieceCameraData data = cam.getValue();
            data.io.updateInputs(data.inputs);
            Logger.processInputs("Inputs/Vision/Gamepiece/" + metadata.name(), data.inputs);
            // Update disconnected alert
            data.disconnectedAlert.set(!data.inputs.connected);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (Map.Entry<AprilTagCamera, AprilTagCameraData> cam : aprilTagCameras.entrySet()) {
            AprilTagCamera metadata = cam.getKey();
            AprilTagCameraData data = cam.getValue();

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : data.inputs.tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                tagPose.ifPresent(tagPoses::add);
            }

            // Loop over pose observations
            for (var observation : data.inputs.poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity if only one tap
                                || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate
                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth()
                                // Reject if station cam and reef cam connected
                                || (metadata == AprilTagCamera.StationCam && aprilTagCameras.get(AprilTagCamera.ReefCam).inputs.connected);

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), metadata.distancePower) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                linearStdDev *= metadata.stddevMultiplier;
                angularStdDev *= metadata.stddevMultiplier;

                // Send vision observation
                robotState.addVisionMeasurement(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            // Log camera data
            String prefix = "Vision/AprilTag/" + metadata.name();
            Logger.recordOutput(prefix + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(prefix + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(prefix + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(prefix + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        // Gamepiece vision
        var robotTranslation = robotState.getTranslation();
        var allGamepieces = new ArrayList<Translation2d>();
        for (Map.Entry<GamepieceCamera, GamepieceCameraData> cam : gamepieceCameras.entrySet()) {
            GamepieceCamera metadata = cam.getKey();
            GamepieceCameraData data = cam.getValue();

            // TODO: replace with algorithm getting closest if there is more than one targets
            var present = data.inputs.latestGamepieceTargetObservation.isPresent();

            String prefix = "Vision/Gamepiece/" + metadata.name();
            Logger.recordOutput(prefix + "/TargetPresent", present);

            if (present) {
                var closestTarget = data.inputs.latestGamepieceTargetObservation.targetPos();
                var closestTargetAbsolute = robotTranslation.plus(closestTarget);

                Logger.recordOutput(
                        prefix + "/ClosestPose",
                        new Pose3d(closestTargetAbsolute.getX(), closestTargetAbsolute.getY(), 0, new Rotation3d()));

                allGamepieces.add(closestTargetAbsolute);
            }
        }
        closestGamepiece = Optional.empty();
        for (var gamepiece : allGamepieces) {
            if (closestGamepiece.isEmpty() || robotTranslation.getDistance(gamepiece) < robotTranslation.getDistance(closestGamepiece.get())) {
                closestGamepiece = Optional.of(gamepiece);
            }
        }
        Logger.recordOutput("Vision/Summary/ClosestGamepiece/Present", closestGamepiece.isPresent());
        closestGamepiece.ifPresent(translation2d -> {
            var asPose3d = new Pose3d(translation2d.getX(), translation2d.getY(), 0, new Rotation3d());
            Logger.recordOutput("Vision/Summary/ClosestGamepiece/Pose", asPose3d);
        });
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera poses for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput(
                "Vision/AprilTagCameraPoses",
                aprilTagCameras.keySet().stream()
                        .map(cam -> robotPose.transformBy(cam.robotToCamera))
                        .toArray(Pose3d[]::new)
        );
        Logger.recordOutput(
                "Vision/GamepieceCameraPoses",
                gamepieceCameras.keySet().stream()
                        .map(cam -> robotPose.transformBy(cam.robotToCamera))
                        .toArray(Pose3d[]::new)
        );
    }

    private record AprilTagCameraData(
            AprilTagIOInputsAutoLogged inputs,
            AprilTagIO io,
            Alert disconnectedAlert
    ) {
    }

    private record GamepieceCameraData(
            GamepieceIOInputsAutoLogged inputs,
            GamepieceIO io,
            Alert disconnectedAlert
    ) {}
}

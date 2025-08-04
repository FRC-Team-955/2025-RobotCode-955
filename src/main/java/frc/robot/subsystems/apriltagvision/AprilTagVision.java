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

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.Util;
import frc.lib.subsystem.Periodic;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

import java.util.*;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

public class AprilTagVision implements Periodic {
    private final RobotState robotState = RobotState.get();

    private final EnumMap<Camera, CameraData> cameras = Util.createEnumMap(Camera.class, Camera.values(), (cam) -> new CameraData(
            new AprilTagVisionIOInputsAutoLogged(),
            cam.createIO(),
            new Alert("AprilTag vision camera " + cam.name() + " is disconnected.", AlertType.kError)
    ));

    private static AprilTagVision instance;

    public static AprilTagVision get() {
        if (instance == null)
            synchronized (AprilTagVision.class) {
                instance = new AprilTagVision();
            }

        return instance;
    }

    private AprilTagVision() {
    }

    @Override
    public void periodicBeforeCommands() {
        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            Camera metadata = cam.getKey();
            CameraData data = cam.getValue();
            data.io.updateInputs(data.inputs);
            Logger.processInputs("Inputs/AprilTagVision/" + metadata.name(), data.inputs);
            // Update disconnected alert
            data.disconnectedAlert.set(!data.inputs.connected);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (Map.Entry<Camera, CameraData> cam : cameras.entrySet()) {
            Camera metadata = cam.getKey();
            CameraData data = cam.getValue();

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (var observation : data.inputs.aprilTagObservations) {
                var tagPose = aprilTagLayout.getTagPose(observation.id());
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                } else {
                    Util.error("Couldn't find tag with ID " + observation.id());
                }
            }

            // Congregate best target and multi tag observations
            List<GenericPoseObservation> genericPoseObservations = new LinkedList<>();

            for (var observation : data.inputs.bestTargetObservations) {
                Optional<Rotation2d> headingSampleOptional = robotState.getPoseAtTimestamp(observation.timestamp()).map(Pose2d::getRotation);

                Optional<Pose3d> tagPoseOptional = aprilTagLayout.getTagPose(observation.tagID());
                if (tagPoseOptional.isEmpty()) {
                    Util.error("Couldn't find tag with ID " + observation.tagID());
                    continue;
                }
                Pose3d tagPose = tagPoseOptional.get();

                double tagDistance = observation.cameraToTarget().getTranslation().getNorm();

                //////////////////////////////// 3d solve ////////////////////////////////
                Transform3d fieldToTarget = new Transform3d(tagPose.getTranslation(), tagPose.getRotation());
                Transform3d fieldToCamera = fieldToTarget.plus(observation.cameraToTarget().inverse());
                Transform3d fieldToRobot = fieldToCamera.plus(metadata.robotToCamera.inverse());
                Pose3d poseEstimate3dSolve = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
                GenericPoseObservation observation3dSolve = new GenericPoseObservation(
                        PoseEsimationMethod.SOLVE_3D,
                        observation.timestamp(),
                        observation.ambiguity(),
                        1,
                        tagDistance,
                        poseEstimate3dSolve,
                        linearStdDevBaseline3dSolveMeters,
                        angularStdDevBaseline3dSolveRad
                );

                if (headingSampleOptional.isPresent()) {
                    //////////////////////////////// Trig ////////////////////////////////
                    // https://github.com/PhotonVision/photonvision/blob/0ef7c803f91a387a1a95377bf64338509218a240/photon-lib/src/main/java/org/photonvision/PhotonPoseEstimator.java#L496
                    Rotation2d headingSample = headingSampleOptional.get();

                    Translation2d camToTagTranslation = new Translation3d(
                            observation.cameraToTarget().getTranslation().getNorm(),
                            new Rotation3d(
                                    0,
                                    -Math.toRadians(observation.pitch()),
                                    -Math.toRadians(observation.yaw())
                            )
                    )
                            .rotateBy(metadata.robotToCamera.getRotation())
                            .toTranslation2d()
                            .rotateBy(headingSample);

                    Translation2d fieldToCameraTranslation = tagPose
                            .toPose2d()
                            .getTranslation()
                            .plus(camToTagTranslation.unaryMinus());

                    Translation2d camToRobotTranslation = metadata.robotToCamera
                            .getTranslation()
                            .toTranslation2d()
                            .unaryMinus()
                            .rotateBy(headingSample);

                    Pose2d poseEstimateTrig = new Pose2d(fieldToCameraTranslation.plus(camToRobotTranslation), headingSample);
                    GenericPoseObservation observationTrig = new GenericPoseObservation(
                            PoseEsimationMethod.TRIG,
                            observation.timestamp(),
                            observation.ambiguity(),
                            1,
                            tagDistance,
                            new Pose3d(poseEstimateTrig),
                            linearStdDevBaselineTrigMeters,
                            angularStdDevBaselineTrigRad
                    );

                    Pose2d poseEstimate3dSolve2d = poseEstimate3dSolve.toPose2d();
                    if (tagDistance < distanceFromTagForTrigMeters &&
                            poseEstimate3dSolve2d.getTranslation().getDistance(poseEstimateTrig.getTranslation()) < trig3dSolveMaxDiffMeters &&
                            Math.abs(poseEstimate3dSolve2d.getRotation().minus(poseEstimateTrig.getRotation()).getRadians()) < trig3dSolveMaxDiffRad) {
                        genericPoseObservations.add(observationTrig);
                    } else {
                        genericPoseObservations.add(observation3dSolve);
                    }
                } else {
                    genericPoseObservations.add(observation3dSolve);
                }
            }

            for (var observation : data.inputs.multiTagObservations) {
                Transform3d fieldToRobot = observation.fieldToCamera().plus(metadata.robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                genericPoseObservations.add(new GenericPoseObservation(
                        PoseEsimationMethod.MULTITAG,
                        observation.timestamp(),
                        observation.ambiguity(),
                        observation.tagCount(),
                        observation.averageTagDistance(),
                        robotPose,
                        linearStdDevBaseline3dSolveMeters,
                        angularStdDevBaseline3dSolveRad
                ));
            }

            // Now that we have congregated best target and multitag observations,
            // we can now filter them and apply them if they are not rejected
            for (var observation : genericPoseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity if only one tap
                                || Math.abs(observation.poseEstimate().getZ()) > maxZError // Must have realistic Z coordinate
                                // Must be within the field boundaries
                                || observation.poseEstimate().getX() < 0.0
                                || observation.poseEstimate().getX() > aprilTagLayout.getFieldLength()
                                || observation.poseEstimate().getY() < 0.0
                                || observation.poseEstimate().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.poseEstimate());
                if (rejectPose) {
                    robotPosesRejected.add(observation.poseEstimate());
                } else {
                    robotPosesAccepted.add(observation.poseEstimate());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), metadata.distancePower) / observation.tagCount();
                double linearStdDev = observation.linearStdDevBaseline * stdDevFactor * metadata.stddevMultiplier;
                double angularStdDev = observation.angularStdDevBaseline * stdDevFactor * metadata.stddevMultiplier;

                // Send vision observation
                robotState.addVisionMeasurement(
                        observation.poseEstimate().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                );
            }

            // Log camera data
            String prefix = "AprilTagVision/" + metadata.name() + "/";
            Logger.recordOutput(prefix + "TagPoses", tagPoses.toArray(Pose3d[]::new));
            Logger.recordOutput(prefix + "GenericPoseObservations", genericPoseObservations.toArray(GenericPoseObservation[]::new));
            Logger.recordOutput(prefix + "RobotPoses", robotPoses.toArray(Pose3d[]::new));
            Logger.recordOutput(prefix + "RobotPosesAccepted", robotPosesAccepted.toArray(Pose3d[]::new));
            Logger.recordOutput(prefix + "RobotPosesRejected", robotPosesRejected.toArray(Pose3d[]::new));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("AprilTagVision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("AprilTagVision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
        Logger.recordOutput("AprilTagVision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
        Logger.recordOutput("AprilTagVision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
    }

    @Override
    public void periodicAfterCommands() {
        // Log camera poses for debugging
        var robotPose = new Pose3d(robotState.getPose());
        Logger.recordOutput(
                "AprilTagVision/CameraPoses",
                Arrays.stream(Camera.values())
                        .map(cam -> robotPose.transformBy(cam.robotToCamera))
                        .toArray(Pose3d[]::new)
        );
    }

    private enum PoseEsimationMethod {
        MULTITAG,
        SOLVE_3D,
        TRIG,
    }

    private record GenericPoseObservation(
            PoseEsimationMethod method,
            double timestamp,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            Pose3d poseEstimate,
            double linearStdDevBaseline,
            double angularStdDevBaseline
    ) {
    }

    private record CameraData(
            AprilTagVisionIOInputsAutoLogged inputs,
            AprilTagVisionIO io,
            Alert disconnectedAlert
    ) {
    }
}

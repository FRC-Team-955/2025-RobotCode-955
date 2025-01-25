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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.RobotState;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import frc.robot.util.SubsystemBaseExt;
import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import static frc.robot.subsystems.vision.VisionConstants.*;

public class Vision extends SubsystemBaseExt {
    private final RobotState robotState = RobotState.get();

    private final VisionIO[] vsIo = visionIO;
    private final GamepieceIO[] gpIo = gamepieceIO;
    private final VisionIOInputsAutoLogged[] vsInputs;
    private final GamepieceIOInputsAutoLogged[] gpInputs;
    private final Alert[] vsDisconnectedAlerts;
    private final Alert[] gpDisconnectedAlerts;

    private static Vision instance;

    private Vision() { // Initialize inputs
        this.vsInputs = new VisionIOInputsAutoLogged[vsIo.length];
        this.gpInputs = new GamepieceIOInputsAutoLogged[gpIo.length];
        for (int i = 0; i < vsInputs.length; i++) {
            vsInputs[i] = new VisionIOInputsAutoLogged();
        }
        for (int i = 0; i < gpInputs.length; i++) {
            gpInputs[i] = new GamepieceIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.vsDisconnectedAlerts = new Alert[vsIo.length];
        for (int i = 0; i < vsInputs.length; i++) {
            vsDisconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + i + " is disconnected.", AlertType.kWarning);
        }

        this.gpDisconnectedAlerts = new Alert[gpIo.length];
        for (int i = 0; i < gpInputs.length; i++) {
            gpDisconnectedAlerts[i] =
                    new Alert(
                            "Gampiece camera " + i + " is disconnected.", AlertType.kWarning);
        }
    }

    public static Vision get() {
        if (instance == null)
            synchronized (Vision.class) {
                instance = new Vision();
            }

        return instance;
    }

    public Optional<Translation2d> closestGamepiece() {
        // TODO: determine closest target in periodic instead of direction using inputs
        return gpInputs[0].latestTargetObservation.isPresent()
                ? Optional.of(robotState.getTranslation().plus(gpInputs[0].latestTargetObservation.targetPos()))
                : Optional.empty();
    }

    @Override
    public void periodicBeforeCommands() {
        for (int i = 0; i < vsIo.length; i++) {
            vsIo[i].updateInputs(vsInputs[i]);
            Logger.processInputs("Inputs/Vision/Camera" + i, vsInputs[i]);
        }

        for (int i = 0; i < gpIo.length; i++) {
            gpIo[i].updateInputs(gpInputs[i]);
            Logger.processInputs("Inputs/Vision/Gamepiece" + i, gpInputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < vsIo.length; cameraIndex++) {
            // Update disconnected alert
            vsDisconnectedAlerts[cameraIndex].set(!vsInputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : vsInputs[cameraIndex].tagIds) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : vsInputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                                || Math.abs(observation.pose().getZ())
                                > maxZError // Must have realistic Z coordinate

                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

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
                double stdDevFactor =
                        Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                RobotState.get().addVisionMeasurement(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + cameraIndex + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        for (int cameraIndex = 0; cameraIndex < gpIo.length; cameraIndex++) {
            // TODO: publish all gamepieces, publish as Pose3d
            Logger.recordOutput("Vision/Gampiece" + cameraIndex + "/RobotRelativePose", closestGamepiece().orElse(new Translation2d()));
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
}

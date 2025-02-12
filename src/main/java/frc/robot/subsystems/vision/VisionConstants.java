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

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotState;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
            new double[]{
                    1.0, // Camera 0
                    1.0 // Camera 1
            };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    public static final GamepieceIO[] gamepieceIO = switch (Constants.identity) {
        /*
        case COMPBOT -> Constants.isReplay
                ? new GamepieceIO[]{new GamepieceIO()}
                : new GamepieceIO[]{new GamepieceIOLimelight("limelight", new Transform3d())};
         */
        case COMPBOT -> new GamepieceIO[]{};
        case ALPHABOT -> Constants.isReplay
                ? new GamepieceIO[]{new GamepieceIO()}
                : new GamepieceIO[]{
                new GamepieceIOLimelight(
                        "limelight",
                        // 2 inches back, 2 inches right, 37 inches up, 40 degrees down from horizontal
                        new Transform3d(Units.inchesToMeters(-2), Units.inchesToMeters(-2), Units.inchesToMeters(37),
                                new Rotation3d(0, Units.degreesToRadians(-40), 0)
                        )
                )
        };
        case SIMBOT -> Constants.isReplay
                ? new GamepieceIO[]{new GamepieceIO()}
                : new GamepieceIO[]{new GamepieceIOSim()};
    };

    public static final AprilTagIO[] aprilTagIO = switch (Constants.identity) {
        case COMPBOT -> Constants.isReplay
                ? new AprilTagIO[]{new AprilTagIO(), new AprilTagIO()}
                : new AprilTagIO[]{
                new AprilTagIOPhotonVision(
                        "camera_0",
                        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0))
                ),
                new AprilTagIOPhotonVision(
                        "camera_1",
                        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI))
                )
        };
//        case ALPHABOT -> Constants.isReplay
//                ? new VisionIO[]{new VisionIO()}
//                : new VisionIO[]{
//                new VisionIOPhotonVision(
//                        "camera_0",
//                        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0))
//                )
//        };
        case ALPHABOT -> new AprilTagIO[]{};
        case SIMBOT -> Constants.isReplay
                ? new AprilTagIO[]{new AprilTagIO(), new AprilTagIO()}
                : new AprilTagIO[]{
                new AprilTagIOPhotonVisionSim(
                        "camera_0",
                        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0)
                        ),
                        RobotState.get()::getPose),
                new AprilTagIOPhotonVisionSim(
                        "camera_1",
                        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI)
                        ),
                        RobotState.get()::getPose)
        };
    };
}

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
import lombok.RequiredArgsConstructor;

import java.util.function.Function;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.25;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.15; // Meters
    public static double angularStdDevBaseline = Units.degreesToRadians(15); // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    @RequiredArgsConstructor
    public enum AprilTagCamera {
        StationCam(
                new Transform3d(
                        Units.inchesToMeters(-6.5), Units.inchesToMeters(-8.375), Units.inchesToMeters(27.5),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(-15), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30)))
                ),
                (cam) -> switch (Constants.identity) {
                    case COMPBOT -> new AprilTagIOPhotonVision("StationCam", cam.robotToCamera);
                    case SIMBOT -> new AprilTagIOPhotonVisionSim("StationCam", cam.robotToCamera);
                    case ALPHABOT -> new AprilTagIO();
                },
                // Relatively stable, even at long distance
                2.0,
                1.0
        ),
        ReefCam(
                new Transform3d(
                        Units.inchesToMeters(-8), Units.inchesToMeters(8.75), Units.inchesToMeters(25.75),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(35), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-170)))
                ),
                (cam) -> switch (Constants.identity) {
                    case COMPBOT -> new AprilTagIOPhotonVision("ReefCam", cam.robotToCamera);
                    case SIMBOT -> new AprilTagIOPhotonVisionSim("ReefCam", cam.robotToCamera);
                    case ALPHABOT -> new AprilTagIO();
                },
                // Trust more at close distance, less at long distance
                2.5,
                0.5
        ),
        ;

        public final Transform3d robotToCamera;
        private final Function<AprilTagCamera, AprilTagIO> createIO;
        public final double distancePower;
        public final double stddevMultiplier;

        public AprilTagIO createIO() {
            if (Constants.isReplay) {
                return new AprilTagIO();
            }

            return createIO.apply(this);
        }
    }

    @RequiredArgsConstructor
    public enum GamepieceCamera {
//        Limelight(
//                // 2 inches back, 2 inches right, 37 inches up, 40 degrees down from horizontal
//                new Transform3d(Units.inchesToMeters(-2), Units.inchesToMeters(-2), Units.inchesToMeters(37),
//                        new Rotation3d(0, Units.degreesToRadians(40), 0)
//                ),
//                (cam) -> switch (Constants.identity) {
//                    case COMPBOT -> new GamepieceIO();
//                    case ALPHABOT -> new GamepieceIOLimelight("limelight", cam.robotToCamera);
//                    case SIMBOT -> new GamepieceIOSim();
//                }
//        ),
        ;

        public final Transform3d robotToCamera;
        private final Function<GamepieceCamera, GamepieceIO> createIO;

        public GamepieceIO createIO() {
            if (Constants.isReplay) {
                return new GamepieceIO();
            }

            return createIO.apply(this);
        }
    }
}

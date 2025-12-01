package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;
import lombok.RequiredArgsConstructor;

import java.util.function.Function;

public class AprilTagVisionConstants {
    // AprilTag layout
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    static final double maxAmbiguity = 0.3;
    static final double maxZError = 0.2;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    static final double linearStdDevBaselineTrigMeters = 0.1;
    static final double angularStdDevBaselineTrigRad = Units.degreesToRadians(30);
    static final double linearStdDevBaseline3dSolveMeters = 0.3;
    static final double angularStdDevBaseline3dSolveRad = Units.degreesToRadians(60);

    // Distance from a tag for trig estimation to be used
    static final double distanceFromTagForTrigMeters = 1.5;
    // Max difference between 3d solve and trig for trig to be used
    static final double trig3dSolveMaxDiffMeters = 0.2;
    static final double trig3dSolveMaxDiffRad = 0.15;

    @RequiredArgsConstructor
    enum Camera {
        StationCam(
                new Transform3d(
                        Units.inchesToMeters(0.0), Units.inchesToMeters(-10), Units.inchesToMeters(30.0),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(30), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90)))
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL, REPLAY -> null;
                    case SIM -> new AprilTagVisionIOPhotonVisionSim("StationCam", cam.robotToCamera);
                },
                // Relatively stable, even at long distance
                2.25,
                1.0
        ),
        ReefCam(
                new Transform3d(
                        // -7.75 x without elevator slant
                        Units.inchesToMeters(0.0), Units.inchesToMeters(10), Units.inchesToMeters(30.0),
                        // Rotation order matters
                        new Rotation3d(0.0, Units.degreesToRadians(30), 0.0)
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(90)))
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL, REPLAY -> null;
                    case SIM -> new AprilTagVisionIOPhotonVisionSim("ReefCam", cam.robotToCamera);
                },
                // Trust more at close distance, less at long distance
                2.25,
                1.0
        ),
        ;

        final Transform3d robotToCamera;
        private final Function<Camera, AprilTagVisionIO> createIO;
        final double distancePower;
        final double stddevMultiplier;

        AprilTagVisionIO createIO() {
            return createIO.apply(this);
        }
    }
}

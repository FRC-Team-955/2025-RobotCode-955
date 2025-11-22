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
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    static final double maxAmbiguity = 0.3;
    static final double maxZError = 0.2;

    static final double linearStdDevBaselineTrigMeters = 0.1;
    static final double angularStdDevBaselineTrigRad = Units.degreesToRadians(30);
    static final double linearStdDevBaseline3dSolveMeters = 0.3;
    static final double angularStdDevBaseline3dSolveRad = Units.degreesToRadians(60);

    static final double distanceFromTagForTrigMeters = 1.5;

    static final double trig3dSolveMaxDiffMeters = 0.2;
    static final double trig3dSolveMaxDiffRad = 0.15;

    @RequiredArgsConstructor
    enum Camera {
        StationCam(
                new Transform3d(
                        Units.inchesToMeters(-6.625),
                        Units.inchesToMeters(-9.125),
                        Units.inchesToMeters(27.25),
                        new Rotation3d(
                                0.0,
                                Units.degreesToRadians(-15),
                                0.0
                        )
                                .rotateBy(
                                        new Rotation3d(
                                                0.0,
                                                0.0,
                                                Units.degreesToRadians(-30)
                                        )
                                )
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL -> new AprilTagVisionIO();
                    case SIM -> new AprilTagVisionIOPhotonVisionSim("StationCam", cam.robotToCamera);
                    case REPLAY -> new AprilTagVisionIO();
                },
                2.0,
                1.0
        ),
        ReefCam(
                new Transform3d(
                        Units.inchesToMeters(-7.5),
                        Units.inchesToMeters(10.25),
                        Units.inchesToMeters(24.75),
                        // Rotation order matters
                        new Rotation3d(
                                Units.degreesToRadians(20), 0.0, 0.0)
                                // 35 pitch without elevator slant
                                .rotateBy(new Rotation3d(0.0, Units.degreesToRadians(30), 0.0))
                                .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(-153.5)))
                ),
                (cam) -> switch (BuildConstants.mode) {
                    case REAL -> new AprilTagVisionIO();
                    case SIM -> new AprilTagVisionIOPhotonVisionSim("ReefCam", cam.robotToCamera);
                    case REPLAY -> new AprilTagVisionIO();
                },
                2.5,
                0.5
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

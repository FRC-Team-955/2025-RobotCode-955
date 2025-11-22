package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.drive.ModuleIOSim;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import java.util.function.Supplier;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.aprilTagLayout;

public class AprilTagVisionIOPhotonVisionSim extends AprilTagVisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private static final Supplier<Pose2d> poseSupplier = ModuleIOSim.driveSimulation::getSimulatedDriveTrainPose;
    private final PhotonCameraSim cameraSim;

    public AprilTagVisionIOPhotonVisionSim(String name, Transform3d robotToCamera) {
        super(name);
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }

        var cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(95.5));
        cameraProperties.setCalibError(5, 3);
        cameraProperties.setFPS(25);
        cameraProperties.setAvgLatencyMs(45);
        cameraProperties.setLatencyStdDevMs(10);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}

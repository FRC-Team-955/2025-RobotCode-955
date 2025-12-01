package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;

import java.util.LinkedList;
import java.util.List;


public class AprilTagVisionIOPhotonVision extends AprilTagVisionIO {
    protected final PhotonCamera camera;

    public AprilTagVisionIOPhotonVision(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        List<AprilTagTargetObservation> aprilTagTargetObservations = new LinkedList<>();
        List<BestTargetObservation> bestTargetObservations = new LinkedList<>();
        List<MultiTagObservation> multiTagObservations = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {
            for (var target : result.getTargets()) {
                aprilTagTargetObservations.add(new AprilTagTargetObservation(
                        result.getTimestampSeconds(),
                        target.getFiducialId(),
                        Rotation2d.fromDegrees(target.getYaw()),
                        Rotation2d.fromDegrees(target.getPitch())
                ));
            }
            if (result.hasTargets()) {
                var bestTarget = result.getBestTarget();

                bestTargetObservations.add(new BestTargetObservation(
                        result.getTimestampSeconds(),
                        bestTarget.poseAmbiguity,
                        bestTarget.fiducialId,
                        bestTarget.bestCameraToTarget,
                        bestTarget.getPitch(),
                        bestTarget.getYaw()
                ));
            }

            if (result.multitagResult.isPresent()) {
                var multitagResult = result.multitagResult.get();

                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                multiTagObservations.add(new MultiTagObservation(
                        result.getTimestampSeconds(),
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size(),
                        totalTagDistance / multitagResult.fiducialIDsUsed.size(),
                        multitagResult.estimatedPose.best
                ));
            }
        }

        inputs.aprilTagObservations = aprilTagTargetObservations.toArray(AprilTagTargetObservation[]::new);
        inputs.bestTargetObservations = bestTargetObservations.toArray(BestTargetObservation[]::new);
        inputs.multiTagObservations = multiTagObservations.toArray(MultiTagObservation[]::new);
    }
}

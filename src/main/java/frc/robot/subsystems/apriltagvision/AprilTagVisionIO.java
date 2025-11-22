package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public class AprilTagVisionIO {
    @AutoLog
    public static class AprilTagVisionIOInputs {
        public boolean connected = false;
        public AprilTagTargetObservation[] aprilTagObservations = new AprilTagTargetObservation[0];
        public BestTargetObservation[] bestTargetObservations = new BestTargetObservation[0];
        public MultiTagObservation[] multiTagObservations = new MultiTagObservation[0];

    }

    public record AprilTagTargetObservation(
            double timestamp,
            int id,
            Rotation2d tx,
            Rotation2d ty
    ) {
    }

    public record BestTargetObservation(
            double timestamp,
            double ambiguity,
            int tagID,
            Transform3d cameraToTarget,
            double pitch,
            double yaw
    ) {
    }

    public record MultiTagObservation(
            double timestamp,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            Transform3d fieldToCamera
    ) {
    }

    public void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}

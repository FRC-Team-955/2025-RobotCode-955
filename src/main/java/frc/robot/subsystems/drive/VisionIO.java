package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public class VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTargets = false;
        public boolean hasEstimatedPose = false;
        public Pose3d estimatedPose = new Pose3d();
        public double timestampSeconds = 0;
        public double bestTargetAmbiguity = 1;
        public double bestTargetArea = 0;
        public int numTargets = 0;
//        public Integer[] targetsList = new Integer[]{};
    }

    public void updateInputs(VisionIOInputs inputs) {
    }
}

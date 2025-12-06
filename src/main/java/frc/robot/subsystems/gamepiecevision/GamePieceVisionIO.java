package frc.robot.subsystems.gamepiecevision;

import org.littletonrobotics.junction.AutoLog;

public class GamePieceVisionIO {
    @AutoLog
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public TargetObservation[] targetObservations = new TargetObservation[0];
    }

    public record TargetObservation(
            double timestamp,
            // rotation order is yaw-pitch (Tait-Bryan angles without roll)
            // left (ccw about z) is positive
            double yawRad,
            // down (ccw about y) is positive
            double pitchRad
    ) {}

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }
}

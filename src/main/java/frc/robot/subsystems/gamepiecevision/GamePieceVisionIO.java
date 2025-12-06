package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public class GamePieceVisionIO {
    @AutoLog
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public TargetObservation[] targetObservations = new TargetObservation[0];
    }

    public record TargetObservation(
            double timestamp,
            // left (ccw about z) is positive
            Rotation2d yaw,
            // down (ccw about y) is positive
            Rotation2d pitch
    ) {}

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }
}

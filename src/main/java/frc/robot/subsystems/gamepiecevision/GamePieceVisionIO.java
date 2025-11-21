package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.AutoLog;

public class GamePieceVisionIO {
    @AutoLog
    public static class GamePieceVisionIOInputs {
        public boolean connected = false;
        public boolean ledsOn = false;
        public boolean visible = false;
        public Pose2d coralPos = new Pose2d();
        public Rotation2d coralYaw = new Rotation2d();
        public Rotation2d coralPitch = new Rotation2d();
    }

    public void updateInputs(GamePieceVisionIOInputs inputs) {
    }

    public void setLEDs(boolean on) {}
}

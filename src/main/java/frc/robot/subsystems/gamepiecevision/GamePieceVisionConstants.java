package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;

public class GamePieceVisionConstants {
    static final Camera camera = new Camera(
            new Transform3d(0, 0, 0.5, new Rotation3d(Units.degreesToRadians(30.0), Units.degreesToRadians(30.0), Units.degreesToRadians(30.0))),
            Units.degreesToRadians(62.5),
            Units.degreesToRadians(48.9)
    );

    static GamePieceVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> null;
            case SIM -> new GamePieceVisionIOSim();
            case REPLAY -> null;
        };
    }

    record Camera(
            Transform3d robotToCamera,
            double horizontalFovRad,
            double verticalFovRad
    ) {}
}

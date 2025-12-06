package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;

public class GamePieceVisionConstants {
    /** Minimum distance between two seen coral for them to be counted as the same coral */
    static final double minDistanceForSameCoral = 0.25;
    static final double seenCoralExpireTime = 3;
    static final double coralHeightMeters = Units.inchesToMeters(4.25);

    static final Camera camera = new Camera(
            new Transform3d(0, 0, 0.5, new Rotation3d()),//new Rotation3d(Units.degreesToRadians(30.0), Units.degreesToRadians(30.0), Units.degreesToRadians(30.0))),
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

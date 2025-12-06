package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.TaitBryanAngles;
import frc.robot.BuildConstants;

public class GamePieceVisionConstants {
    /** Minimum distance between two seen coral for them to be counted as the same coral */
    static final double minDistanceForSameCoralMeters = 1;
    static final double seenCoralExpireTimeSeconds = 5;
    static final double coralHeightMeters = Units.inchesToMeters(4.25);
    static final double seenCoralTimeForRecent = 0.5;

    static final Camera camera = new Camera(
            new Transform3d(
                    new Translation3d(0, 0, 0.5),
                    new Rotation3d(0, Units.degreesToRadians(30.0), 0)
                            .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(30.0)))
            ),
            Units.degreesToRadians(62.5),
            Units.degreesToRadians(48.9)
    );
    static final TaitBryanAngles cameraOrientation = new TaitBryanAngles(camera.robotToCamera().getRotation().getQuaternion());

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

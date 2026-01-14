package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.TaitBryanAngles;
import frc.robot.BuildConstants;

import static frc.robot.subsystems.drive.DriveConstants.driveConfig;

public class GamePieceVisionConstants {
    /** Minimum distance between two seen coral for them to be counted as the same coral */
    static final double minDistanceForSameCoralMeters = 1;
    static final double staleExpireTimeSeconds = 3;
    static final double coralHeightMeters = Units.inchesToMeters(4.25);
    static final double freshExpireTimeSeconds = 0.5;

    static final Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                    Units.inchesToMeters(8.452489),
                    Units.inchesToMeters(9.467625),
                    // Carpet to bottom of frame
                    (driveConfig.wheelRadiusMeters() - Units.inchesToMeters(0.247775))
                            // Bottom of frame to camera
                            + Units.inchesToMeters(8.604877)
            ),
            new Rotation3d(0, Units.degreesToRadians(25.0), 0)
                    .rotateBy(new Rotation3d(0, 0, Units.degreesToRadians(-20.0)))
            // .rotateBy(new Rotation3d(.2, 0, 0)) // TODO fix roll comp
    );
    static final TaitBryanAngles cameraOrientation = new TaitBryanAngles(robotToCamera.getRotation().getQuaternion());

    static GamePieceVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new GamePieceVisionIOLimelight("limelight");
            case SIM -> new GamePieceVisionIOSim(
                    Units.degreesToRadians(62.5),
                    Units.degreesToRadians(48.9)
            );
            case REPLAY -> new GamePieceVisionIO();
        };
    }
}

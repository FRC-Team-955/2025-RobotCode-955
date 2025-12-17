package frc.robot.subsystems.gamepiecevision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.BuildConstants;

public class GamePieceVisionConstants {
    static double halfFOVRad = Math.PI / 3;
    static final double cameraPitch = Units.degreesToRadians(25);
    static final double cameraYaw = Units.degreesToRadians(20);
    static final Transform3d camToRobot = new Transform3d(
            Units.inchesToMeters(8.487707),
            Units.inchesToMeters(-9.454807),
            Units.inchesToMeters(-10.314625),
            new Rotation3d(0, cameraPitch, cameraYaw)
    );

    static GamePieceVisionIO createIO() {
        return switch (BuildConstants.mode) {
            case REAL -> new GamePieceVisionIOLimelight("limelight");
            case SIM -> new GamePieceVisionIOSim();
            case REPLAY -> new GamePieceVisionIO();
        };
    }
}
